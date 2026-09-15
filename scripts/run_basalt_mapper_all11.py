#!/usr/bin/env python3
"""Detached, resumable driver: run the ported Basalt NFR offline mapper on
all 11 EuRoC sequences, one at a time, then evaluate keyframe-only and
full-trajectory ATE against ground truth.

Designed to be launched detached (e.g. via PowerShell Start-Process) so it
survives the launching shell exiting. Resumable: each sequence writes a
status/<SEQ>.json marker on success; a rerun skips any sequence whose marker
already exists. A sequence that fails writes status/<SEQ>.failed.json and the
driver moves on to the next sequence (does not abort the whole run).

Runs are strictly sequential (one mapper process at a time) because each run
holds several GB of RSS (embedded per-keyframe images + BoW descriptor DB +
sparse BA state) and the box does not have headroom for 11 in parallel.

Usage:
    python scripts/run_basalt_mapper_all11.py
    python scripts/run_basalt_mapper_all11.py --seq MH_01_easy   # single seq
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
BIN = Path(
    "E:/visloc-rs-runs/basalt_goal_mapper_identity_20260913/target-msvc/"
    "release/examples/basalt_mapper_offline_demo.exe"
)
CALIB = REPO / "benchmarks/basalt/release_inputs/euroc_ds_calib.json"
CONFIG = REPO / "configs/basalt/euroc_config.json"
VIO_MARG_ROOT = Path("E:/visloc-rs-runs/basalt_lc_ceiling_20260914/vio_marg")
GT_ROOT = Path("E:/datasets/euroc_mav/all11")
OUT_ROOT = Path("E:/visloc-rs-runs/basalt_mapper_all11_20260915")

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

RSS_KILL_BYTES = 12 * 1024 * 1024 * 1024  # 12 GiB safety kill, per task instructions.
WALL_KILL_SECONDS = 60 * 60  # 60 min per-sequence safety kill.


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

    def __init__(self, proc, seq):
        self.proc = proc
        self.seq = seq
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
                if rss > RSS_KILL_BYTES:
                    self.killed_reason = f"RSS {rss / 1e9:.2f} GB exceeded {RSS_KILL_BYTES / 1e9:.0f} GB cap"
                    log_line(f"[{self.seq}] KILLING: {self.killed_reason}")
                    self.proc.kill()
                    break
            elapsed = time.time() - self._started
            if elapsed > WALL_KILL_SECONDS:
                self.killed_reason = f"wall time {elapsed:.0f}s exceeded {WALL_KILL_SECONDS}s cap"
                log_line(f"[{self.seq}] KILLING: {self.killed_reason}")
                self.proc.kill()
                break
            self._stop.wait(5.0)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=10)


def run_mapper(seq):
    out_dir = OUT_ROOT / "mapper_out" / seq
    out_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = out_dir / "stdout.log"
    stderr_path = out_dir / "stderr.log"
    marg_dir = VIO_MARG_ROOT / seq / "marg_data"
    cmd = [
        str(BIN),
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
    started = time.time()
    with open(stdout_path, "w", encoding="utf-8") as so, open(
        stderr_path, "w", encoding="utf-8"
    ) as se:
        proc = subprocess.Popen(cmd, stdout=so, stderr=se)
        monitor = RssMonitor(proc, seq)
        monitor.start()
        ret = proc.wait()
        monitor.stop()
    elapsed = time.time() - started
    if monitor.killed_reason is not None:
        raise RuntimeError(f"{seq}: killed by safety monitor ({monitor.killed_reason})")
    if ret != 0:
        tail = stderr_path.read_text(encoding="utf-8", errors="replace")[-2000:]
        raise RuntimeError(f"{seq}: mapper exited with code {ret}\n--- stderr tail ---\n{tail}")
    log_line(
        f"[{seq}] mapper finished in {elapsed:.1f}s, peak_rss={monitor.peak_bytes / 1e9:.2f} GB"
    )
    return {"wall_seconds": elapsed, "peak_rss_bytes": monitor.peak_bytes}


def run_propagate_and_eval(seq):
    out_dir = OUT_ROOT / "mapper_out" / seq
    eval_dir = OUT_ROOT / "eval" / seq
    eval_dir.mkdir(parents=True, exist_ok=True)
    vio_csv = VIO_MARG_ROOT / seq / "trajectory.csv"
    poses_json = out_dir / "poses.json"
    full_tum = out_dir / "trajectory_full_propagated.tum"

    subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts/propagate_basalt_mapper_corrections.py"),
            "--vio-trajectory-csv",
            str(vio_csv),
            "--mapper-poses-json",
            str(poses_json),
            "--out-tum",
            str(full_tum),
        ],
        check=True,
    )

    gt_csv = GT_ROOT / seq / "mav0/state_groundtruth_estimate0/data.csv"
    kf_json = eval_dir / "kf_ate.json"
    full_json = eval_dir / "full_ate.json"
    subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts/evaluate_euroc_trajectory.py"),
            "--ground-truth-csv",
            str(gt_csv),
            "--trajectory",
            str(out_dir / "trajectory.tum"),
            "--tum-time-unit",
            "s",
            "--out-json",
            str(kf_json),
        ],
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts/evaluate_euroc_trajectory.py"),
            "--ground-truth-csv",
            str(gt_csv),
            "--trajectory",
            str(full_tum),
            "--tum-time-unit",
            "s",
            "--out-json",
            str(full_json),
        ],
        check=True,
    )
    kf_run = json.load(open(kf_json, encoding="utf-8"))["runs"][0]
    full_run = json.load(open(full_json, encoding="utf-8"))["runs"][0]
    return kf_run, full_run


def run_one_sequence(seq):
    status_dir = OUT_ROOT / "status"
    status_dir.mkdir(parents=True, exist_ok=True)
    done_marker = status_dir / f"{seq}.json"
    failed_marker = status_dir / f"{seq}.failed.json"
    if done_marker.exists():
        log_line(f"[{seq}] already done, skipping")
        return
    if failed_marker.exists():
        failed_marker.unlink()

    try:
        timing = run_mapper(seq)
        report = json.load(
            open(OUT_ROOT / "mapper_out" / seq / "mapper_report.json", encoding="utf-8")
        )
        kf_ate, full_ate = run_propagate_and_eval(seq)
        record = {
            "sequence": seq,
            "wall_seconds": timing["wall_seconds"],
            "peak_rss_bytes": timing["peak_rss_bytes"],
            "pose_count": report["result"]["pose_count"],
            "landmark_count": report["result"]["landmark_count"],
            "final_point_count": report["result"]["final_point_count"],
            "match_all": report["match_all"],
            "tracks": report["tracks"],
            "first_optimize": report["first_optimize"],
            "second_optimize": report["second_optimize"],
            "kf_ate_se3_rmse_m": kf_ate["ate_translation_se3_m"]["rmse"],
            "kf_ate_sim3_rmse_m": kf_ate["ate_translation_sim3_m"]["rmse"],
            "kf_associated_poses": kf_ate["associated_poses"],
            "kf_estimate_poses": kf_ate["estimate_poses"],
            "full_ate_se3_rmse_m": full_ate["ate_translation_se3_m"]["rmse"],
            "full_ate_sim3_rmse_m": full_ate["ate_translation_sim3_m"]["rmse"],
            "full_associated_poses": full_ate["associated_poses"],
            "full_estimate_poses": full_ate["estimate_poses"],
            "completed_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(done_marker, "w", encoding="utf-8") as stream:
            json.dump(record, stream, indent=2)
        log_line(
            f"[{seq}] DONE kf_ate={record['kf_ate_se3_rmse_m']:.4f}m "
            f"full_ate={record['full_ate_se3_rmse_m']:.4f}m "
            f"wall={record['wall_seconds']:.1f}s peak_rss={record['peak_rss_bytes']/1e9:.2f}GB"
        )
    except Exception as error:  # noqa: BLE001 - record and continue to the next sequence.
        failure = {
            "sequence": seq,
            "error": str(error),
            "traceback": traceback.format_exc(),
            "failed_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(failed_marker, "w", encoding="utf-8") as stream:
            json.dump(failure, stream, indent=2)
        log_line(f"[{seq}] FAILED: {error}")


def write_summary():
    status_dir = OUT_ROOT / "status"
    rows = []
    for seq in SEQUENCES:
        done = status_dir / f"{seq}.json"
        failed = status_dir / f"{seq}.failed.json"
        if done.exists():
            rows.append(json.load(open(done, encoding="utf-8")))
        elif failed.exists():
            rows.append(json.load(open(failed, encoding="utf-8")))
        else:
            rows.append({"sequence": seq, "status": "not_run"})

    summary_json = OUT_ROOT / "summary.json"
    with open(summary_json, "w", encoding="utf-8") as stream:
        json.dump({"generated_at": datetime.now(timezone.utc).isoformat(), "rows": rows}, stream, indent=2)

    lines = [
        "# Basalt NFR offline mapper — all-11 EuRoC results",
        "",
        "| Seq | VIO ATE | Mapper KF ATE | Mapper Full ATE | Stage-A PG | ORB-SLAM3 | Wall | Peak RSS | Wins vs ORB-SLAM3 |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        seq = row["sequence"]
        if "error" in row:
            lines.append(f"| {seq} | FAILED: {row['error'][:80]} | | | | | | | |")
            continue
        if "wall_seconds" not in row:
            lines.append(f"| {seq} | not run | | | | | | | |")
            continue
        lines.append(
            f"| {seq} | | {row['kf_ate_se3_rmse_m']:.4f} | {row['full_ate_se3_rmse_m']:.4f} | | | "
            f"{row['wall_seconds']/60:.1f} min | {row['peak_rss_bytes']/1e9:.2f} GB | |"
        )
    with open(OUT_ROOT / "summary.md", "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    log_line(f"wrote {summary_json} and summary.md")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seq", action="append", default=None, help="restrict to these sequences")
    args = parser.parse_args()
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "mapper_out").mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "eval").mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "status").mkdir(parents=True, exist_ok=True)

    sequences = args.seq if args.seq else SEQUENCES
    log_line(f"driver starting, sequences={sequences}")
    for seq in sequences:
        run_one_sequence(seq)
    write_summary()
    log_line("driver finished all sequences")


if __name__ == "__main__":
    main()
