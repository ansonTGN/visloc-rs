#!/usr/bin/env python3
"""Detached, resumable driver: run Basalt VIO (with MargData) then the native
NFR offline mapper on all 11 EuRoC sequences, one process at a time, using
the a-priori official-calibration Double Sphere variant
(configs/basalt/variants/official_euroc_ds/), then evaluate VIO and mapper
KF/full-trajectory SE3+Sim3 ATE against ground truth and against the
upstream-noise baseline (E:\\visloc-rs-runs\\basalt_lc_ceiling_20260914\\vio_marg
for VIO, E:\\visloc-rs-runs\\basalt_mapper_all11_20260915\\status for the mapper).

Calibration provenance / caveat (verbatim, carry into any report that reads
this driver's output): the official_euroc_ds calibration's Double Sphere
intrinsics were fit by least squares to EuRoC's official pinhole-radtan
model (mav0/cam{0,1}/sensor.yaml), restricted to the inner 90% of the
undistorted bearing radius (cam0 fit RMS 0.222 px / max 0.397 px; cam1 RMS
0.211 px / max 0.377 px -- both below Basalt's own vio_obs_std_dev=0.5 px
observation noise). The outer ~10% radius (the literal image corners) was
excluded because EuRoC's 2-coefficient radtan (k1, k2, no k3) is extrapolated
there well past where real checkerboard corners were ever detected during
the original factory calibration -- those corner residuals (up to 13-16 px
in the unrestricted fit) are not information this conversion should
reproduce, and FAST corner features are essentially never detected there
either. The fit's xi converges to ~0 (a degenerate near-pinhole shape) in
this region -- expected and accepted: this variant reproduces the *official*
factory model, not Basalt's own recalibrated Double Sphere shape (xi=-0.24
cam0 / -0.21 cam1). T_imu_cam comes directly from the official T_BS
matrices (same body<-sensor direction as Basalt's IMU<-cam convention).
IMU noise/bias fields and cam_time_offset_ns are unchanged from the release
calibration (benchmarks/basalt/release_inputs/euroc_ds_calib.json).

Designed to be launched detached (PowerShell Start-Process) so it survives
the launching shell exiting. Resumable: each stage writes a status marker;
a rerun skips any (seq, stage) whose marker already exists. A sequence that
fails a stage writes a `<seq>.<stage>.failed.json` marker and the driver
moves on to the next sequence (does not abort the whole run). Only one VIO
or mapper process runs at a time.

Usage:
    python scripts/run_basalt_official_calib_all11.py
    python scripts/run_basalt_official_calib_all11.py --seq MH_01_easy
"""

import argparse
import csv
import io
import json
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
VIO_BIN = Path(
    "E:/visloc-rs-runs/basalt_goal_mapper_identity_20260913/target-msvc/"
    "release/examples/basalt_euroc_vio_demo.exe"
)
MAPPER_BIN = Path(
    "E:/visloc-rs-runs/basalt_goal_mapper_identity_20260913/target-msvc/"
    "release/examples/basalt_mapper_offline_demo.exe"
)
CALIB = REPO / "configs/basalt/variants/official_euroc_ds/euroc_ds_calib.json"
CONFIG = REPO / "configs/basalt/variants/official_euroc_ds/euroc_config.json"
PROPAGATE_SCRIPT = Path(
    "E:/visloc-rs-runs/mapper_all11_wt/scripts/propagate_basalt_mapper_corrections.py"
)
EVAL_SCRIPT = REPO / "scripts/evaluate_euroc_trajectory.py"

EUROC_SENSOR_ONLY_ROOT = Path("E:/datasets/euroc_mav/sensor_only_all11")
GT_ROOT = Path("E:/datasets/euroc_mav/all11")
OUT_ROOT = Path("E:/visloc-rs-runs/basalt_official_calib_20260915")

BASELINE_VIO_MARG_ROOT = Path("E:/visloc-rs-runs/basalt_lc_ceiling_20260914/vio_marg")
BASELINE_MAPPER_STATUS_ROOT = Path("E:/visloc-rs-runs/basalt_mapper_all11_20260915/status")

SEQUENCES = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
]

VIO_RSS_KILL_BYTES = 4 * 1024 * 1024 * 1024  # 4 GiB safety kill for the VIO stage.
MAPPER_RSS_KILL_BYTES = 12 * 1024 * 1024 * 1024  # 12 GiB safety kill for the mapper stage.
WALL_KILL_SECONDS = 60 * 60  # 60 min per process safety kill.


def log_line(message):
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    line = f"[{stamp}] {message}"
    print(line, flush=True)
    with open(OUT_ROOT / "driver.log", "a", encoding="utf-8") as stream:
        stream.write(line + "\n")


def sample_rss_bytes(pid):
    try:
        out = subprocess.run(
            ["tasklist", "/FI", f"PID eq {pid}", "/FO", "CSV", "/NH"],
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout.strip()
        if not out or "No tasks" in out or "INFO:" in out:
            return None
        row = next(csv.reader(io.StringIO(out)))
        mem_str = row[4].replace(",", "").replace(" K", "").strip()
        return int(mem_str) * 1024
    except Exception:
        return None


class RssMonitor:
    """Background sampler: peak RSS + wall/RSS kill switch for one process."""

    def __init__(self, proc, label, rss_kill_bytes):
        self.proc = proc
        self.label = label
        self.rss_kill_bytes = rss_kill_bytes
        self.peak_bytes = 0
        self.killed_reason = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._started = time.time()

    def _run(self):
        while not self._stop.is_set():
            rss = sample_rss_bytes(self.proc.pid)
            if rss is not None:
                self.peak_bytes = max(self.peak_bytes, rss)
                if rss > self.rss_kill_bytes:
                    self.killed_reason = f"RSS {rss / 1e9:.2f} GB exceeded {self.rss_kill_bytes / 1e9:.0f} GB cap"
                    log_line(f"[{self.label}] KILLING: {self.killed_reason}")
                    self.proc.kill()
                    break
            elapsed = time.time() - self._started
            if elapsed > WALL_KILL_SECONDS:
                self.killed_reason = f"wall time {elapsed:.0f}s exceeded {WALL_KILL_SECONDS}s cap"
                log_line(f"[{self.label}] KILLING: {self.killed_reason}")
                self.proc.kill()
                break
            self._stop.wait(5.0)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=10)


def run_subprocess(cmd, stdout_path, stderr_path, label, rss_kill_bytes):
    started = time.time()
    with open(stdout_path, "w", encoding="utf-8") as so, open(stderr_path, "w", encoding="utf-8") as se:
        proc = subprocess.Popen(cmd, stdout=so, stderr=se)
        monitor = RssMonitor(proc, label, rss_kill_bytes)
        monitor.start()
        ret = proc.wait()
        monitor.stop()
    elapsed = time.time() - started
    if monitor.killed_reason is not None:
        raise RuntimeError(f"{label}: killed by safety monitor ({monitor.killed_reason})")
    if ret != 0:
        tail = stderr_path.read_text(encoding="utf-8", errors="replace")[-2000:]
        raise RuntimeError(f"{label}: exited with code {ret}\n--- stderr tail ---\n{tail}")
    return {"wall_seconds": elapsed, "peak_rss_bytes": monitor.peak_bytes}


def run_vio(seq):
    out_dir = OUT_ROOT / "vio_marg" / seq
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(VIO_BIN),
        "--euroc-dir",
        str(EUROC_SENSOR_ONLY_ROOT / seq),
        "--calibration",
        str(CALIB),
        "--config",
        str(CONFIG),
        "--out-dir",
        str(out_dir),
        "--no-trace",
    ]
    log_line(f"[{seq}] launching VIO (MargData enabled): {' '.join(cmd)}")
    timing = run_subprocess(
        cmd, out_dir / "stdout.log", out_dir / "stderr.log", f"{seq}/vio", VIO_RSS_KILL_BYTES
    )
    log_line(f"[{seq}] VIO finished in {timing['wall_seconds']:.1f}s, peak_rss={timing['peak_rss_bytes']/1e9:.2f} GB")
    return timing


def run_mapper(seq):
    out_dir = OUT_ROOT / "mapper_out" / seq
    out_dir.mkdir(parents=True, exist_ok=True)
    marg_dir = OUT_ROOT / "vio_marg" / seq / "marg_data"
    cmd = [
        str(MAPPER_BIN),
        "--marg-dir",
        str(marg_dir),
        "--calibration",
        str(CALIB),
        "--config",
        str(CONFIG),
        "--out-dir",
        str(out_dir),
    ]
    log_line(f"[{seq}] launching mapper: {' '.join(cmd)}")
    timing = run_subprocess(
        cmd, out_dir / "stdout.log", out_dir / "stderr.log", f"{seq}/mapper", MAPPER_RSS_KILL_BYTES
    )
    log_line(f"[{seq}] mapper finished in {timing['wall_seconds']:.1f}s, peak_rss={timing['peak_rss_bytes']/1e9:.2f} GB")
    return timing


def evaluate(seq, trajectory_tum, out_json, tum_time_unit="s"):
    gt_csv = GT_ROOT / seq / "mav0/state_groundtruth_estimate0/data.csv"
    subprocess.run(
        [
            sys.executable,
            str(EVAL_SCRIPT),
            "--ground-truth-csv",
            str(gt_csv),
            "--trajectory",
            str(trajectory_tum),
            "--tum-time-unit",
            tum_time_unit,
            "--out-json",
            str(out_json),
        ],
        check=True,
    )
    return json.load(open(out_json, encoding="utf-8"))["runs"][0]


def run_propagate_and_eval(seq):
    out_dir = OUT_ROOT / "mapper_out" / seq
    eval_dir = OUT_ROOT / "eval" / seq
    eval_dir.mkdir(parents=True, exist_ok=True)
    vio_csv = OUT_ROOT / "vio_marg" / seq / "trajectory.csv"
    poses_json = out_dir / "poses.json"
    full_tum = out_dir / "trajectory_full_propagated.tum"

    subprocess.run(
        [
            sys.executable,
            str(PROPAGATE_SCRIPT),
            "--vio-trajectory-csv",
            str(vio_csv),
            "--mapper-poses-json",
            str(poses_json),
            "--out-tum",
            str(full_tum),
        ],
        check=True,
    )

    kf_run = evaluate(seq, out_dir / "trajectory.tum", eval_dir / "mapper_kf_ate.json")
    full_run = evaluate(seq, full_tum, eval_dir / "mapper_full_ate.json")
    return kf_run, full_run


def run_one_sequence(seq):
    status_dir = OUT_ROOT / "status"
    status_dir.mkdir(parents=True, exist_ok=True)
    vio_done = status_dir / f"{seq}.vio.json"
    vio_failed = status_dir / f"{seq}.vio.failed.json"
    mapper_done = status_dir / f"{seq}.json"
    mapper_failed = status_dir / f"{seq}.failed.json"

    if mapper_done.exists():
        log_line(f"[{seq}] already fully done, skipping")
        return

    # --- VIO stage ---
    if vio_done.exists():
        log_line(f"[{seq}] VIO already done, skipping VIO stage")
        vio_record = json.load(open(vio_done, encoding="utf-8"))
    else:
        if vio_failed.exists():
            vio_failed.unlink()
        try:
            timing = run_vio(seq)
            vio_eval_dir = OUT_ROOT / "eval" / seq
            vio_eval_dir.mkdir(parents=True, exist_ok=True)
            vio_ate = evaluate(
                seq,
                OUT_ROOT / "vio_marg" / seq / "trajectory.tum",
                vio_eval_dir / "vio_ate.json",
            )
            vio_record = {
                "sequence": seq,
                "wall_seconds": timing["wall_seconds"],
                "peak_rss_bytes": timing["peak_rss_bytes"],
                "vio_se3_rmse_m": vio_ate["ate_translation_se3_m"]["rmse"],
                "vio_sim3_rmse_m": vio_ate["ate_translation_sim3_m"]["rmse"],
                "vio_sim3_scale": vio_ate["sim3_scale"],
                "vio_associated_poses": vio_ate["associated_poses"],
                "completed_at": datetime.now(timezone.utc).isoformat(),
            }
            with open(vio_done, "w", encoding="utf-8") as stream:
                json.dump(vio_record, stream, indent=2)
            log_line(
                f"[{seq}] VIO DONE se3={vio_record['vio_se3_rmse_m']:.4f}m "
                f"sim3={vio_record['vio_sim3_rmse_m']:.4f}m scale={vio_record['vio_sim3_scale']:.4f}"
            )
        except Exception as error:  # noqa: BLE001
            failure = {
                "sequence": seq,
                "stage": "vio",
                "error": str(error),
                "traceback": traceback.format_exc(),
                "failed_at": datetime.now(timezone.utc).isoformat(),
            }
            with open(vio_failed, "w", encoding="utf-8") as stream:
                json.dump(failure, stream, indent=2)
            log_line(f"[{seq}] VIO FAILED: {error}")
            return  # cannot run the mapper without VIO MargData

    # --- Mapper stage ---
    if mapper_failed.exists():
        mapper_failed.unlink()
    try:
        timing = run_mapper(seq)
        report = json.load(open(OUT_ROOT / "mapper_out" / seq / "mapper_report.json", encoding="utf-8"))
        kf_ate, full_ate = run_propagate_and_eval(seq)
        record = {
            "sequence": seq,
            "vio": vio_record,
            "mapper_wall_seconds": timing["wall_seconds"],
            "mapper_peak_rss_bytes": timing["peak_rss_bytes"],
            "pose_count": report["result"]["pose_count"],
            "landmark_count": report["result"]["landmark_count"],
            "final_point_count": report["result"]["final_point_count"],
            "kf_ate_se3_rmse_m": kf_ate["ate_translation_se3_m"]["rmse"],
            "kf_ate_sim3_rmse_m": kf_ate["ate_translation_sim3_m"]["rmse"],
            "kf_sim3_scale": kf_ate["sim3_scale"],
            "full_ate_se3_rmse_m": full_ate["ate_translation_se3_m"]["rmse"],
            "full_ate_sim3_rmse_m": full_ate["ate_translation_sim3_m"]["rmse"],
            "full_sim3_scale": full_ate["sim3_scale"],
            "completed_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(mapper_done, "w", encoding="utf-8") as stream:
            json.dump(record, stream, indent=2)
        log_line(
            f"[{seq}] MAPPER DONE kf_se3={record['kf_ate_se3_rmse_m']:.4f}m "
            f"full_se3={record['full_ate_se3_rmse_m']:.4f}m "
            f"wall={record['mapper_wall_seconds']/60:.1f}min "
            f"peak_rss={record['mapper_peak_rss_bytes']/1e9:.2f}GB"
        )
    except Exception as error:  # noqa: BLE001
        failure = {
            "sequence": seq,
            "stage": "mapper",
            "error": str(error),
            "traceback": traceback.format_exc(),
            "failed_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(mapper_failed, "w", encoding="utf-8") as stream:
            json.dump(failure, stream, indent=2)
        log_line(f"[{seq}] MAPPER FAILED: {error}")


def baseline_vio(seq):
    tum = BASELINE_VIO_MARG_ROOT / seq / "trajectory.tum"
    if not tum.exists():
        return None
    out_json = OUT_ROOT / "eval" / seq / "baseline_vio_ate.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    try:
        run = evaluate(seq, tum, out_json)
        return {
            "se3": run["ate_translation_se3_m"]["rmse"],
            "sim3": run["ate_translation_sim3_m"]["rmse"],
            "scale": run["sim3_scale"],
        }
    except Exception:
        return None


def baseline_mapper(seq):
    status = BASELINE_MAPPER_STATUS_ROOT / f"{seq}.json"
    if not status.exists():
        return None
    row = json.load(open(status, encoding="utf-8"))
    if "kf_ate_se3_rmse_m" not in row:
        return None
    return {
        "kf_se3": row["kf_ate_se3_rmse_m"],
        "kf_sim3": row["kf_ate_sim3_rmse_m"],
        "full_se3": row["full_ate_se3_rmse_m"],
        "full_sim3": row["full_ate_sim3_rmse_m"],
    }


def write_summary():
    status_dir = OUT_ROOT / "status"
    rows = []
    for seq in SEQUENCES:
        done = status_dir / f"{seq}.json"
        vio_done = status_dir / f"{seq}.vio.json"
        vio_failed = status_dir / f"{seq}.vio.failed.json"
        mapper_failed = status_dir / f"{seq}.failed.json"
        base_vio = baseline_vio(seq)
        base_mapper = baseline_mapper(seq)
        if done.exists():
            row = json.load(open(done, encoding="utf-8"))
        elif vio_failed.exists():
            row = json.load(open(vio_failed, encoding="utf-8"))
        elif mapper_failed.exists():
            row = json.load(open(mapper_failed, encoding="utf-8"))
        elif vio_done.exists():
            row = {"sequence": seq, "vio": json.load(open(vio_done, encoding="utf-8")), "status": "vio_only"}
        else:
            row = {"sequence": seq, "status": "not_run"}
        row["baseline_vio"] = base_vio
        row["baseline_mapper"] = base_mapper
        rows.append(row)

    summary_json = OUT_ROOT / "summary.json"
    with open(summary_json, "w", encoding="utf-8") as stream:
        json.dump({"generated_at": datetime.now(timezone.utc).isoformat(), "rows": rows}, stream, indent=2)

    lines = [
        "# Basalt official-calibration (inner-90% DS fit) all-11 EuRoC results",
        "",
        "Calibration: configs/basalt/variants/official_euroc_ds/ "
        "(DS fit to official pinhole-radtan restricted to inner 90% bearing "
        "radius; cam0 fit RMS 0.222px, cam1 0.211px; xi~=0 by construction, "
        "reproducing the official model rather than Basalt's own DS shape).",
        "",
        "| Seq | VIO SE3 (base->official) | VIO Sim3 (base->official) | VIO scale (base->official) "
        "| Mapper KF SE3 (base->official) | Mapper Full SE3 (base->official) | Wall (VIO+mapper) |",
        "|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        seq = row["sequence"]
        base_vio = row.get("baseline_vio") or {}
        base_mapper = row.get("baseline_mapper") or {}
        vio = row.get("vio") if "vio" in row else None
        if row.get("status") == "not_run":
            lines.append(f"| {seq} | not run | | | | | |")
            continue
        if "error" in row:
            lines.append(f"| {seq} | FAILED ({row.get('stage', '?')}): {row['error'][:80]} | | | | | |")
            continue
        vio = vio or {}
        vio_se3 = f"{base_vio.get('se3', float('nan')):.4f}->{vio.get('vio_se3_rmse_m', float('nan')):.4f}"
        vio_sim3 = f"{base_vio.get('sim3', float('nan')):.4f}->{vio.get('vio_sim3_rmse_m', float('nan')):.4f}"
        vio_scale = f"{base_vio.get('scale', float('nan')):.4f}->{vio.get('vio_sim3_scale', float('nan')):.4f}"
        if row.get("status") == "vio_only" or "kf_ate_se3_rmse_m" not in row:
            lines.append(f"| {seq} | {vio_se3} | {vio_sim3} | {vio_scale} | (mapper not run) | | |")
            continue
        kf_se3 = f"{base_mapper.get('kf_se3', float('nan')):.4f}->{row['kf_ate_se3_rmse_m']:.4f}"
        full_se3 = f"{base_mapper.get('full_se3', float('nan')):.4f}->{row['full_ate_se3_rmse_m']:.4f}"
        wall = f"{(vio.get('wall_seconds', 0) + row.get('mapper_wall_seconds', 0)) / 60:.1f} min"
        lines.append(f"| {seq} | {vio_se3} | {vio_sim3} | {vio_scale} | {kf_se3} | {full_se3} | {wall} |")

    with open(OUT_ROOT / "summary.md", "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    log_line(f"wrote {summary_json} and summary.md")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seq", action="append", default=None, help="restrict to these sequences")
    args = parser.parse_args()
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "vio_marg").mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "mapper_out").mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "eval").mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "status").mkdir(parents=True, exist_ok=True)

    sequences = args.seq if args.seq else SEQUENCES
    log_line(f"driver starting, sequences={sequences}")
    for seq in sequences:
        run_one_sequence(seq)
        write_summary()  # refresh after every sequence so progress is visible mid-run
    write_summary()
    log_line("driver finished all sequences")


if __name__ == "__main__":
    main()
