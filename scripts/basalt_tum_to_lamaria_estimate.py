#!/usr/bin/env python3
"""Convert a Basalt-port TUM trajectory to the LaMAria estimate format.

Our TUM writers (`append_trajectory` in
examples/basalt_euroc_vio_demo.rs, and `write_tum` in
scripts/propagate_basalt_mapper_corrections.py) both emit

    timestamp_seconds tx ty tz qx qy qz qw

with `timestamp_seconds` a float printed to 9 decimal places (i.e.
nanosecond-resolution, since every source timestamp is an integer
nanosecond count divided by 1e9) and the pose being `state.imu_to_world`
(world_from_imu) -- exactly the convention `cvg/lamaria`'s evaluators
expect (see lamaria/structs/trajectory.py: `Trajectory.load_from_file`
with `invert_poses=False` treats each row as world_from_imu already).

The one format gap is the timestamp: LaMAria's evaluation scripts require
"The timestamp must be in nanoseconds" (integer), not float seconds. This
script does the one substantive conversion (round(seconds * 1e9) -> int)
and otherwise passes tx,ty,tz,qx,qy,qz,qw through unchanged, one line per
input line, space-separated, no header -- LaMAria's own
`Trajectory.load_from_file` parser (space-split, 8 columns) accepts this
directly.

Usage:
    python scripts/basalt_tum_to_lamaria_estimate.py \
        --in-tum target/lamaria_R_01_easy_mapper/full_trajectory.tum \
        --out-estimate target/lamaria_R_01_easy_mapper/R_01_easy_estimate.txt
"""

import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--in-tum", type=Path, required=True, help="Basalt-port TUM file (seconds tx ty tz qx qy qz qw)")
    parser.add_argument("--out-estimate", type=Path, required=True, help="LaMAria estimate file (ns tx ty tz qx qy qz qw)")
    args = parser.parse_args()

    lines_out = []
    n_in = 0
    previous_ts_ns = None
    non_monotonic = 0
    for line in args.in_tum.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        n_in += 1
        fields = stripped.split()
        if len(fields) != 8:
            raise SystemExit(
                f"{args.in_tum}:{n_in}: expected 8 space-separated fields "
                f"(timestamp_s tx ty tz qx qy qz qw), got {len(fields)}: {stripped!r}"
            )
        timestamp_s = float(fields[0])
        timestamp_ns = round(timestamp_s * 1.0e9)
        if previous_ts_ns is not None and timestamp_ns <= previous_ts_ns:
            non_monotonic += 1
        previous_ts_ns = timestamp_ns
        rest = " ".join(fields[1:])
        lines_out.append(f"{timestamp_ns} {rest}")

    if n_in == 0:
        raise SystemExit(f"{args.in_tum}: no pose rows found")
    if non_monotonic:
        print(
            f"[warn] {non_monotonic}/{n_in} rows are non-monotonic or duplicate "
            "after rounding to nanoseconds"
        )

    args.out_estimate.parent.mkdir(parents=True, exist_ok=True)
    args.out_estimate.write_text("\n".join(lines_out) + "\n", encoding="utf-8")
    print(f"wrote {len(lines_out)} poses -> {args.out_estimate}")


if __name__ == "__main__":
    main()
