#!/usr/bin/env python3
"""Detached, resumable driver: run the online Basalt VI-SLAM demo
(examples/basalt_euroc_online_slam_demo.rs -- VIO thread + concurrent mapper
thread, see docs/basalt_online_mapper_design.md) on all 11 EuRoC sequences,
one process at a time, then evaluate the propagated trajectory_online.tum
against ground truth and compare against the offline mapper (§1.4 of
docs/vi_slam_global_consistency_plan.md, E:\\visloc-rs-runs\\
basalt_official_calib_20260915\\status) and measured ORB-SLAM3
(E:\\visloc-rs-runs\\orbslam3_euroc_20260914\\summary.json).

Designed to be launched detached (PowerShell Start-Process / nohup) so it
survives the launching shell exiting. Resumable: each sequence writes a
status marker under <out_root>/status/; a rerun skips any sequence whose
marker already exists. A sequence that fails writes a `<seq>.failed.json`
marker and the driver moves on to the next sequence.

Uses --as-fast-as-possible (default) pacing for the headline RTF/ATE table;
the demo's --realtime paced mode is evaluated separately (MH_01 queue-lag
measurement), not part of this sweep.

Usage:
    python scripts/run_basalt_online_all11.py
    python scripts/run_basalt_online_all11.py --seq MH_01_easy
"""

import argparse
import json
import subprocess
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ONLINE_BIN = Path(
    "E:/visloc-rs-runs/basalt_goal_mapper_identity_20260913/target-msvc/"
    "release/examples/basalt_euroc_online_slam_demo.exe"
)
CALIB = REPO / "configs/basalt/variants/official_euroc_ds/euroc_ds_calib.json"
CONFIG = REPO / "configs/basalt/variants/official_euroc_ds/euroc_config.json"
EVAL_SCRIPT = REPO / "scripts/evaluate_euroc_trajectory.py"

EUROC_SENSOR_ONLY_ROOT = Path("E:/datasets/euroc_mav/sensor_only_all11")
GT_ROOT = Path("E:/datasets/euroc_mav/all11")
OUT_ROOT = Path("E:/visloc-rs-runs/basalt_online_20260915")

OFFLINE_STATUS_ROOT = Path("E:/visloc-rs-runs/basalt_official_calib_20260915/status")
ORBSLAM3_SUMMARY = Path("E:/visloc-rs-runs/orbslam3_euroc_20260914/summary.json")

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

WALL_KILL_SECONDS = 2 * 60 * 60  # 2h per-sequence safety kill (VIO alone was
# ~12-18 min single-threaded offline; online adds concurrent mapper work
# that should now mostly overlap the VIO's wall time with an unbounded
# channel + inverted-index matching + coarse optimize cadence, but this cap
# stays generous rather than tuned to the as-fast-as-possible case).
OPTIMIZE_EVERY_K = 100
PERIODIC_ITERATIONS = 4


def log_line(message):
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    line = f"[{stamp}] {message}"
    print(line, flush=True)
    with open(OUT_ROOT / "driver.log", "a", encoding="utf-8") as stream:
        stream.write(line + "\n")


def run_online_demo(seq):
    out_dir = OUT_ROOT / "runs" / seq
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(ONLINE_BIN),
        "--euroc-dir",
        str(EUROC_SENSOR_ONLY_ROOT / seq),
        "--calibration",
        str(CALIB),
        "--config",
        str(CONFIG),
        "--out-dir",
        str(out_dir),
        "--optimize-every-k",
        str(OPTIMIZE_EVERY_K),
        "--periodic-iterations",
        str(PERIODIC_ITERATIONS),
    ]
    log_line(f"[{seq}] launching online demo: {' '.join(cmd)}")
    started = datetime.now(timezone.utc)
    with open(out_dir / "stdout.log", "w", encoding="utf-8") as so, open(
        out_dir / "stderr.log", "w", encoding="utf-8"
    ) as se:
        try:
            proc = subprocess.run(cmd, stdout=so, stderr=se, timeout=WALL_KILL_SECONDS)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                f"{seq}: killed after exceeding {WALL_KILL_SECONDS}s wall-time cap"
            ) from error
    elapsed = (datetime.now(timezone.utc) - started).total_seconds()
    if proc.returncode != 0:
        tail = (out_dir / "stderr.log").read_text(encoding="utf-8", errors="replace")[-2000:]
        raise RuntimeError(f"{seq}: online demo exited with code {proc.returncode}\n--- stderr tail ---\n{tail}")
    return elapsed, out_dir


def evaluate(seq, trajectory_tum, out_json):
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
            "s",
            "--out-json",
            str(out_json),
        ],
        check=True,
    )
    return json.load(open(out_json, encoding="utf-8"))["runs"][0]


def run_one_sequence(seq):
    status_dir = OUT_ROOT / "status"
    status_dir.mkdir(parents=True, exist_ok=True)
    done = status_dir / f"{seq}.json"
    failed = status_dir / f"{seq}.failed.json"

    if done.exists():
        log_line(f"[{seq}] already done, skipping")
        return

    if failed.exists():
        failed.unlink()
    try:
        driver_wall_seconds, out_dir = run_online_demo(seq)
        timing = json.load(open(out_dir / "timing_breakdown_online.json", encoding="utf-8"))
        eval_dir = OUT_ROOT / "eval" / seq
        eval_dir.mkdir(parents=True, exist_ok=True)
        ate = evaluate(seq, out_dir / "trajectory_online.tum", eval_dir / "online_ate.json")
        record = {
            "sequence": seq,
            "driver_wall_seconds": driver_wall_seconds,
            "vio_wall_seconds": timing["vio_wall_seconds"],
            "mapper_join_seconds": timing["mapper_join_seconds"],
            "total_wall_seconds": timing["total_wall_seconds"],
            "real_time_factor": timing["real_time_factor"],
            "mapper_queue_lag_seconds": timing["mapper_queue_lag_seconds"],
            "mapper_packets_sent": timing["mapper_packets_sent"],
            "mapper": timing["mapper"],
            "peak_working_set_bytes": timing["peak_working_set_bytes"],
            "online_se3_rmse_m": ate["ate_translation_se3_m"]["rmse"],
            "online_sim3_rmse_m": ate["ate_translation_sim3_m"]["rmse"],
            "online_sim3_scale": ate["sim3_scale"],
            "associated_poses": ate["associated_poses"],
            "estimate_poses": ate["estimate_poses"],
            "completed_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(done, "w", encoding="utf-8") as stream:
            json.dump(record, stream, indent=2)
        log_line(
            f"[{seq}] DONE online_se3={record['online_se3_rmse_m']:.4f}m "
            f"rtf={record['real_time_factor']:.3f} "
            f"loops={record['mapper']['accepted_loop_pair_count']} "
            f"optimizes={record['mapper']['optimize_pass_count']} "
            f"peak_rss={record['peak_working_set_bytes'] / 1e6:.0f}MB "
            f"wall={record['total_wall_seconds'] / 60:.1f}min"
        )
    except Exception as error:  # noqa: BLE001
        failure = {
            "sequence": seq,
            "error": str(error),
            "traceback": traceback.format_exc(),
            "failed_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(failed, "w", encoding="utf-8") as stream:
            json.dump(failure, stream, indent=2)
        log_line(f"[{seq}] FAILED: {error}")


def offline_full_se3(seq):
    status = OFFLINE_STATUS_ROOT / f"{seq}.json"
    if not status.exists():
        return None
    row = json.load(open(status, encoding="utf-8"))
    return row.get("full_ate_se3_rmse_m")


def orbslam3_se3(seq):
    if not ORBSLAM3_SUMMARY.exists():
        return None
    summary = json.load(open(ORBSLAM3_SUMMARY, encoding="utf-8"))
    row = summary.get("sequences", {}).get(seq)
    if row is None:
        return None
    return row.get("ate_se3_median_m")


def write_summary():
    status_dir = OUT_ROOT / "status"
    rows = []
    online_wins = 0
    offline_wins = 0
    for seq in SEQUENCES:
        done = status_dir / f"{seq}.json"
        failed = status_dir / f"{seq}.failed.json"
        offline = offline_full_se3(seq)
        orbslam3 = orbslam3_se3(seq)
        if done.exists():
            row = json.load(open(done, encoding="utf-8"))
            row["offline_se3_rmse_m"] = offline
            row["orbslam3_se3_rmse_m"] = orbslam3
            online_se3 = row["online_se3_rmse_m"]
            if orbslam3 is not None:
                row["win_vs_orbslam3"] = online_se3 < orbslam3
                if row["win_vs_orbslam3"]:
                    online_wins += 1
            if offline is not None and offline > 0:
                row["online_vs_offline_ratio"] = online_se3 / offline
                if online_se3 <= offline * 1.10:
                    offline_wins += 1
        elif failed.exists():
            row = json.load(open(failed, encoding="utf-8"))
            row["offline_se3_rmse_m"] = offline
            row["orbslam3_se3_rmse_m"] = orbslam3
        else:
            row = {"sequence": seq, "status": "not_run", "offline_se3_rmse_m": offline, "orbslam3_se3_rmse_m": orbslam3}
        rows.append(row)

    summary_json = OUT_ROOT / "summary.json"
    with open(summary_json, "w", encoding="utf-8") as stream:
        json.dump(
            {
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "online_wins_vs_orbslam3": online_wins,
                "within_10pct_of_offline": offline_wins,
                "rows": rows,
            },
            stream,
            indent=2,
        )

    lines = [
        "# Basalt online mapper: all-11 EuRoC results",
        "",
        "Online (this driver) / offline mapper (§1.4, PR #147) / ORB-SLAM3 "
        "measured, all same official-calibration / same-protocol SE(3) ATE "
        "RMSE (full propagated trajectory).",
        "",
        "| Seq | Online SE3 | Offline SE3 | ORB-SLAM3 SE3 | Win vs ORB-SLAM3 | RTF | Peak RSS | Loops | Optimizes |",
        "|---|---:|---:|---:|:---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        seq = row["sequence"]
        if row.get("status") == "not_run":
            lines.append(f"| {seq} | not run | {fmt(row.get('offline_se3_rmse_m'))} | {fmt(row.get('orbslam3_se3_rmse_m'))} | | | | | |")
            continue
        if "error" in row:
            lines.append(f"| {seq} | FAILED: {row['error'][:80]} | {fmt(row.get('offline_se3_rmse_m'))} | {fmt(row.get('orbslam3_se3_rmse_m'))} | | | | | |")
            continue
        win = "YES" if row.get("win_vs_orbslam3") else "no"
        rtf = f"{row['real_time_factor']:.3f}"
        rss = f"{row['peak_working_set_bytes'] / 1e6:.0f} MB"
        loops = row["mapper"]["accepted_loop_pair_count"]
        optimizes = row["mapper"]["optimize_pass_count"]
        lines.append(
            f"| {seq} | {row['online_se3_rmse_m']:.4f} | {fmt(row.get('offline_se3_rmse_m'))} | "
            f"{fmt(row.get('orbslam3_se3_rmse_m'))} | {win} | {rtf} | {rss} | {loops} | {optimizes} |"
        )

    with open(OUT_ROOT / "summary.md", "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    log_line(f"wrote {summary_json} and summary.md ({online_wins}/11 vs ORB-SLAM3, {offline_wins}/11 within 10% of offline)")


def fmt(value):
    return f"{value:.4f}" if isinstance(value, (int, float)) else "n/a"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seq", action="append", default=None, help="restrict to these sequences")
    args = parser.parse_args()
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUT_ROOT / "runs").mkdir(parents=True, exist_ok=True)
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
