#!/usr/bin/env python3
"""Compare the native link-0 intermediate oracle with the Rust step dump."""

from __future__ import annotations

import re
import struct
import sys
from pathlib import Path


FIELD_RE = re.compile(r"(?:^| )([A-Za-z_][A-Za-z0-9_]*)=([^ ]+)")


def fields(line: str) -> dict[str, str]:
    return {name: value for name, value in FIELD_RE.findall(line)}


def words(value: str) -> list[int]:
    return [int(item, 16) for item in value.split(",") if item]


def first_diff(left: list[int], right: list[int]):
    for index, (lhs, rhs) in enumerate(zip(left, right)):
        if lhs != rhs:
            return index, lhs, rhs
    if len(left) != len(right):
        return min(len(left), len(right)), None, None
    return None


def f32_bits(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", value))[0]


def f32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def native_clean_words(path: Path) -> list[int]:
    result: list[int] = []
    in_cov = False
    for line in path.read_text().splitlines():
        if "M7HD cov 81 float words" in line:
            in_cov = True
            continue
        if in_cov and line.startswith("M7HD COV RETURN"):
            in_cov = False
            continue
        if in_cov:
            result.extend(int(word, 16) for word in re.findall(r"0x([0-9a-fA-F]{8})(?:\s|$)", line))
    return result


def native_clean_matrices(path: Path) -> list[list[int]]:
    """Read each of the four clean link covariance blocks."""
    result: list[list[int]] = []
    current: list[int] | None = None
    for line in path.read_text().splitlines():
        if line.startswith("M7HD cov 81 float words"):
            if current is not None:
                result.append(current)
            current = []
        elif current is not None:
            current.extend(int(word, 16) for word in re.findall(r"0x([0-9a-fA-F]{8})(?:\s|$)", line))
    if current is not None:
        result.append(current)
    return result


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    native_path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "target/m7hd_native_intermediate.jsonl"
    rust_path = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "target/m7hd_step_probe.stdout"
    clean_path = Path(sys.argv[3]) if len(sys.argv) > 3 else root / "target/m7hd_clean_cov_capture.out"
    rust_input_path = Path(sys.argv[4]) if len(sys.argv) > 4 else root / "target/m7hd_input_probe.stdout"
    rust_q_path = Path(sys.argv[5]) if len(sys.argv) > 5 else root / "target/m7hd_q_probe.stdout"
    rust_g_path = Path(sys.argv[6]) if len(sys.argv) > 6 else rust_path
    rust_noise_path = Path(sys.argv[7]) if len(sys.argv) > 7 else None

    native_lines = native_path.read_text().splitlines()
    native_steps = [fields(line) for line in native_lines if line.startswith("M7HD_NATIVE_STEP")]
    native_cov = [fields(line) for line in native_lines if line.startswith("M7HD_NATIVE_COV")]
    rust_all_steps = [fields(line) for line in rust_path.read_text().splitlines() if line.startswith("RUST_M7HD_STEP")]
    rust_steps = rust_all_steps
    rust_g = [fields(line) for line in rust_g_path.read_text().splitlines() if line.startswith("RUST_M7HD_G")]
    rust_noise = (
        [fields(line) for line in rust_noise_path.read_text().splitlines() if line.startswith("RUST_M7HD_NOISE")]
        if rust_noise_path is not None
        else []
    )
    rust_inputs = [fields(line) for line in rust_input_path.read_text().splitlines() if line.startswith("RUST_M7HD_INPUT")]
    rust_q = [fields(line) for line in rust_q_path.read_text().splitlines() if line.startswith("RUST_M7HD_Q")]
    if len(native_steps) != 10 or len(native_cov) != 10:
        raise SystemExit(f"expected 10 native step/cov records, got {len(native_steps)}/{len(native_cov)}")
    if len(rust_steps) < 10:
        raise SystemExit(f"expected at least 10 Rust records, got {len(rust_steps)}")
    rust_steps = rust_steps[:10]
    if len(rust_inputs) < 10:
        raise SystemExit(f"expected at least 10 Rust input records, got {len(rust_inputs)}")
    rust_inputs = rust_inputs[:10]
    if len(rust_q) < 10:
        raise SystemExit(f"expected at least 10 Rust quaternion records, got {len(rust_q)}")
    rust_q = rust_q[:10]
    if len(rust_g) < 10:
        raise SystemExit(f"expected at least 10 Rust G records, got {len(rust_g)}")
    rust_g = rust_g[:10]
    if rust_noise_path is not None:
        if len(rust_noise) < 10:
            raise SystemExit(f"expected at least 10 Rust noise records, got {len(rust_noise)}")
        rust_noise = rust_noise[:10]

    print(f"native_steps={len(native_steps)} native_cov={len(native_cov)} rust_steps={len(rust_steps)}")
    dt_mismatches = [
        (
            i + 1,
            native_steps[i]["dt_bits"],
            f32_bits(f32(int(rust_steps[i]["dt_ns"])) * f32(1.0e-9)),
        )
        for i in range(10)
        if int(native_steps[i]["dt_bits"], 16)
        != f32_bits(f32(int(rust_steps[i]["dt_ns"])) * f32(1.0e-9))
    ]
    print(f"dt_mismatches={len(dt_mismatches)}")
    if dt_mismatches:
        print(f"first_dt_mismatch={dt_mismatches[0]}")

    for input_name in ("accel", "gyro"):
        mismatches = 0
        witness = None
        for packet in range(10):
            diff = first_diff(words(rust_inputs[packet][input_name]), words(native_steps[packet][input_name]))
            if diff is not None:
                mismatches += 1
                if witness is None:
                    witness = (packet + 1, diff[0], diff[1], diff[2])
        print(f"input_{input_name}_mismatches={mismatches}/10")
        if witness is not None:
            print(f"input_{input_name}_first=(packet={witness[0]},lane={witness[1]},rust={witness[2]},native={witness[3]})")

    for q_name in ("pre", "exp", "post"):
        mismatches = 0
        witness = None
        for packet in range(10):
            diff = first_diff(words(rust_q[packet][q_name]), words(native_steps[packet]["q_" + q_name]))
            if diff is not None:
                mismatches += sum(
                    lhs != rhs
                    for lhs, rhs in zip(
                        words(rust_q[packet][q_name]),
                        words(native_steps[packet]["q_" + q_name]),
                    )
                )
                if witness is None:
                    witness = (packet + 1, diff[0], diff[1], diff[2])
        print(f"q_{q_name}_mismatches={mismatches}/40")
        if witness is not None:
            print(f"q_{q_name}_first=(packet={witness[0]},lane={witness[1]},rust={witness[2]},native={witness[3]})")

    stages = ["r_half", "r_new", "jr", "jr2", "f", "a", "g"]
    stages += ["term1", "term2", "term3", "cov"]
    first = None
    for stage in stages:
        mismatches = 0
        exact_lanes = 0
        witness = None
        for packet in range(10):
            if stage in ("term1", "term2", "term3", "cov"):
                native_value = words(native_cov[packet][stage])
                rust_value = words(rust_steps[packet][stage])
            else:
                native_value = words(native_steps[packet][stage])
                rust_value = words(rust_steps[packet][stage])
            diff = first_diff(rust_value, native_value)
            if diff is None:
                exact_lanes += len(native_value)
            else:
                mismatches += sum(lhs != rhs for lhs, rhs in zip(rust_value, native_value))
                if witness is None:
                    lane, rust_word, native_word = diff
                    witness = (packet + 1, lane, rust_word, native_word)
                if first is None:
                    first = (stage, packet + 1, diff[0], diff[1], diff[2])
        total = 10 * len(words(native_cov[0][stage])) if stage in ("term1", "term2", "term3", "cov") else 10 * len(words(native_steps[0][stage]))
        print(f"{stage}: mismatches={mismatches}/{total} exact={total - mismatches}")
        if witness is not None:
            print(f"{stage}_first=(packet={witness[0]},lane={witness[1]},rust={witness[2]},native={witness[3]})")

    g_stages = [
        "g_upper_product",
        "g_upper_dt",
        "g_lower_frot_rhalf",
        "g_lower_jr2",
        "g_lower_half",
        "g_lower_dt",
        "g_position_scale",
    ]
    g_first = None
    for stage in g_stages:
        mismatches = 0
        witness = None
        for packet in range(10):
            native_value = words(native_steps[packet][stage])
            rust_value = words(rust_g[packet][stage.removeprefix("g_")])
            diff = first_diff(rust_value, native_value)
            if diff is not None:
                mismatches += sum(lhs != rhs for lhs, rhs in zip(rust_value, native_value))
                if witness is None:
                    lane, rust_word, native_word = diff
                    witness = (packet + 1, lane, rust_word, native_word)
                if g_first is None:
                    g_first = (stage, packet + 1, diff[0], diff[1], diff[2])
        total = 10 * len(words(native_steps[0][stage]))
        print(f"{stage}: mismatches={mismatches}/{total} exact={total - mismatches}")
        if witness is not None:
            print(f"{stage}_first=(packet={witness[0]},lane={witness[1]},rust={witness[2]},native={witness[3]})")

    if g_first is not None:
        print(
            "g_chronological_first="
            f"stage:{g_first[0]} packet:{g_first[1]} lane:{g_first[2]} "
            f"rust:{g_first[3]:08x} native:{g_first[4]:08x}"
        )
    else:
        print("g_chronological_first=none")

    if rust_noise_path is not None:
        noise_stages = [
            ("accel_scaled", "accel_scaled"),
            ("gyro_scaled", "gyro_scaled"),
            ("term2", "term2"),
            ("term2_scaled", "term2_scaled"),
            ("term3", "term3"),
            ("term3_scaled", "term3_scaled"),
        ]
        noise_first = None
        for stage, native_stage in noise_stages:
            mismatches = 0
            witness = None
            for packet in range(10):
                native_value = words(native_cov[packet][native_stage])
                rust_value = words(rust_noise[packet][stage])
                diff = first_diff(rust_value, native_value)
                if diff is not None:
                    mismatches += sum(lhs != rhs for lhs, rhs in zip(rust_value, native_value))
                    if witness is None:
                        lane, rust_word, native_word = diff
                        witness = (packet + 1, lane, rust_word, native_word)
                    if noise_first is None:
                        noise_first = (stage, packet + 1, diff[0], diff[1], diff[2])
            total = 10 * len(words(native_cov[0][native_stage]))
            print(f"{stage}: mismatches={mismatches}/{total} exact={total - mismatches}")
            if witness is not None:
                print(f"{stage}_first=(packet={witness[0]},lane={witness[1]},rust={witness[2]},native={witness[3]})")
        if noise_first is not None:
            print(
                "noise_chronological_first="
                f"stage:{noise_first[0]} packet:{noise_first[1]} lane:{noise_first[2]} "
                f"rust:{noise_first[3]:08x} native:{noise_first[4]:08x}"
            )
        else:
            print("noise_chronological_first=none")

    chronological = None
    for packet in range(10):
        for stage in stages:
            if stage in ("term1", "term2", "term3", "cov"):
                native_value = words(native_cov[packet][stage])
                rust_value = words(rust_steps[packet][stage])
            else:
                native_value = words(native_steps[packet][stage])
                rust_value = words(rust_steps[packet][stage])
            diff = first_diff(rust_value, native_value)
            if diff is not None:
                chronological = (stage, packet + 1, diff[0], diff[1], diff[2])
                break
        if chronological is not None:
            break

    if chronological is not None:
        print(
            "chronological_first_divergence="
            f"stage:{chronological[0]} packet:{chronological[1]} "
            f"lane:{chronological[2]} rust:{chronological[3]:08x} "
            f"native:{chronological[4]:08x}"
        )
    elif first is not None:
        print(f"chronological_first_divergence=stage:{first[0]} packet:{first[1]} lane:{first[2]} rust:{first[3]:08x} native:{first[4]:08x}")
    else:
        print("chronological_first_divergence=none")

    clean = native_clean_words(clean_path)
    final_native = words(native_cov[9]["cov"])
    clean_link0 = clean[:81]
    diff = first_diff(final_native, clean_link0)
    print(f"native_final_vs_clean_cov_mismatches={sum(lhs != rhs for lhs, rhs in zip(final_native, clean_link0))}/81")
    if diff is not None:
        print(f"native_final_vs_clean_first=(lane={diff[0]},native_probe={diff[1]:08x},clean={diff[2]:08x})")

    clean_links = native_clean_matrices(clean_path)
    if len(clean_links) >= 4 and len(rust_all_steps) >= 40:
        link_counts = []
        for link in range(4):
            rust_final = words(rust_all_steps[(link + 1) * 10 - 1]["cov"])
            clean_final = clean_links[link]
            mismatch = sum(lhs != rhs for lhs, rhs in zip(rust_final, clean_final))
            link_counts.append(mismatch)
        print(
            "rust_final_vs_clean_cov_links="
            + ",".join(f"link{link}={count}/81" for link, count in enumerate(link_counts))
        )
        print(f"rust_final_vs_clean_cov_total={sum(link_counts)}/324")

    print(f"native_accel_cov={native_cov[0]['accel_cov']}")
    print(f"native_gyro_cov={native_cov[0]['gyro_cov']}")
    print(f"native_dt_bits={native_steps[0]['dt_bits']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
