#!/usr/bin/env python3
"""Diagnostic M7 whitening/product schedule fixture.

This is intentionally outside the production Rust crate.  It consumes the
bit-preserving frame-4 fields emitted by ``m7im_cov_ldlt_oracle`` and evaluates
the candidate schedules described in ``m7im_whitening_schedule_report``.  The
result is a machine-readable record of each candidate's IEEE-754 f32 bits and
its comparison with the oracle's four post-whitening fields:

* ``W*r``
* ``W*J``
* ``J^T*J``
* ``J^T*r``

An optional injected Rust JSON can be supplied to compare its exact f32
boundary fields as well.  No production source is imported or modified.  The
fixture uses the platform C ``fmaf`` entry point so that the candidate FMA
operations are rounded once, like Eigen's ``pmadd`` path.  A platform without
an exported ``fmaf`` is rejected rather than silently substituting a widened
Python multiply/add.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import struct
from pathlib import Path
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ORACLE = ROOT / "target/m7im_cov_ldlt_oracle_20260824.json"
DEFAULT_REPORT = ROOT / "target/m7im_whitening_schedule_fixture_20260824.json"


def f32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def bits(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def f32_from_bits(value: Any) -> float:
    if isinstance(value, str):
        return struct.unpack("<f", struct.pack("<I", int(value, 16)))[0]
    return f32(float(value))


def bit_words(values: list[float]) -> list[str]:
    return [f"{bits(value):08x}" for value in values]


def load_fmaf() -> Callable[[float, float, float], float]:
    """Load C fmaf with float arguments/results and fail closed if absent."""

    names = ["ucrtbase.dll", "libm.so.6", "libm.so", "libSystem.B.dylib"]
    for name in names:
        try:
            library = ctypes.CDLL(name)
            function = library.fmaf
            function.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
            function.restype = ctypes.c_float

            def call(a: float, b: float, c: float,
                     function: Any = function) -> float:
                return float(function(ctypes.c_float(a), ctypes.c_float(b),
                                      ctypes.c_float(c)))

            # Keep the library alive through the closure.  The local binding
            # also documents which ABI supplied the arithmetic in provenance.
            call._library_name = name  # type: ignore[attr-defined]
            return call
        except (AttributeError, OSError):
            continue
    raise RuntimeError(
        "no exported C fmaf found; refusing to claim an exact f32 schedule")


FMA = load_fmaf()
FMA_LIBRARY = getattr(FMA, "_library_name", "unknown")


def scalar_mul_add(a: float, b: float, c: float) -> float:
    return f32(f32(a * b) + c)


def fma_sum(values_a: list[float], values_b: list[float]) -> float:
    result = 0.0
    for left, right in zip(values_a, values_b):
        result = FMA(left, right, result)
    return result


def mul_add_sum(values_a: list[float], values_b: list[float]) -> float:
    result = 0.0
    for left, right in zip(values_a, values_b):
        result = scalar_mul_add(left, right, result)
    return result


def read_array(field: Any) -> list[float]:
    """Read an oracle bit vector or legacy decimal nested vector."""

    if isinstance(field, dict):
        return [f32_from_bits(item) for item in field["bits_row_major"]]
    if isinstance(field, list):
        result: list[float] = []
        for item in field:
            if isinstance(item, list):
                result.extend(read_array(item))
            else:
                result.append(f32_from_bits(item))
        return result
    raise TypeError(f"unsupported array field {type(field).__name__}")


def read_matrix(field: Any) -> list[list[float]]:
    if not isinstance(field, dict):
        raise TypeError("matrix field must be an object with dimensions")
    rows = int(field["rows"])
    cols = int(field["cols"])
    values = read_array(field)
    if len(values) != rows * cols:
        raise ValueError(f"matrix has {len(values)} values, expected {rows * cols}")
    return [values[row * cols:(row + 1) * cols] for row in range(rows)]


def matrix_bits(matrix: list[list[float]]) -> list[str]:
    return bit_words([value for row in matrix for value in row])


def vector_bits(vector: list[float]) -> list[str]:
    return bit_words(vector)


def matrix_object(matrix: list[list[float]]) -> dict[str, Any]:
    rows = len(matrix)
    cols = len(matrix[0]) if rows else 0
    return {
        "rows": rows,
        "cols": cols,
        "storage_order": "column_major",
        "bits_row_major": matrix_bits(matrix),
    }


def vector_object(vector: list[float]) -> dict[str, Any]:
    return {"length": len(vector), "bits": vector_bits(vector)}


def compare_words(expected: list[str], actual: list[str]) -> dict[str, Any]:
    mismatch = [index for index, (left, right) in enumerate(zip(expected, actual))
                if left.lower() != right.lower()]
    length_delta = abs(len(expected) - len(actual))
    first = mismatch[0] if mismatch else (min(len(expected), len(actual))
                                          if length_delta else None)
    return {
        "expected_count": len(expected),
        "actual_count": len(actual),
        "mismatch_count": len(mismatch) + length_delta,
        "first_diff": (
            {"index": first, "expected": expected[first],
             "actual": actual[first]}
            if first is not None and first < len(expected) and first < len(actual)
            else ({"index": first, "expected": None, "actual": None}
                  if first is not None else None)
        ),
        "exact": not mismatch and not length_delta,
    }


def oracle_field_bits(field: Any) -> list[str]:
    if isinstance(field, dict):
        return [str(item).lower() for item in field["bits_row_major"]]
    return bit_words(read_array(field))


def rust_link_fields(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    """Select the injected Rust frame-3 -> frame-4 exact f32 fields."""

    value = json.loads(path.read_text())
    mapping = {
        "sqrt_information": "sqrt_information",
        "raw_residual": "raw_residual_f32_exact",
        "jacobian": "jacobian_f32_exact",
        "whitened_residual": "whitened_residual",
        "whitened_jacobian": "whitened_jacobian",
        "row_h": "row_h",
        "row_b": "row_b",
    }
    if "links" in value:
        links = value["links"]
        if not isinstance(links, list) or len(links) <= 3:
            raise ValueError("Rust JSON has no link index 3")
        link = links[3]
    elif "frame4_factor" in value:
        link = value["frame4_factor"]
    else:
        raise ValueError("Rust JSON must contain links or frame4_factor")
    missing = [name for name in mapping.values() if name not in link]
    if missing:
        raise ValueError("Rust JSON missing exact fields: " + ", ".join(missing))
    return link, mapping


def gemv_w_r_fma(w: list[list[float]], r: list[float]) -> list[float]:
    result = [0.0] * 9
    for row in range(9):
        for k in range(9):
            result[row] = FMA(w[row][k], r[k], result[row])
    return result


def gemv_w_r_packet(w: list[list[float]], r: list[float]) -> list[float]:
    # The packet rows are independent lanes, but the scalar model has the
    # same k order.  Keep the tail explicit for schedule provenance.
    result = [0.0] * 9
    for row in range(8):
        for k in range(9):
            result[row] = FMA(w[row][k], r[k], result[row])
    result[8] = fma_sum(w[8], r)
    return result


def gemv_w_r_mul_add(w: list[list[float]], r: list[float]) -> list[float]:
    return [mul_add_sum(row, r) for row in w]


def gemm_w_j_fma(w: list[list[float]], j: list[list[float]]) -> list[list[float]]:
    return [[fma_sum([w[row][k] for k in range(9)],
                      [j[k][col] for k in range(9)])
             for col in range(30)] for row in range(9)]


def gebp_row8_tree(a: list[float], b: list[float]) -> float:
    pair0 = FMA(a[0], b[0], 0.0) + FMA(a[1], b[1], 0.0)
    pair1 = FMA(a[2], b[2], 0.0) + FMA(a[3], b[3], 0.0)
    pair2 = FMA(a[4], b[4], 0.0) + FMA(a[5], b[5], 0.0)
    pair3 = FMA(a[6], b[6], 0.0) + FMA(a[7], b[7], 0.0)
    # Python's ``+`` is widened unless the operands are explicitly narrowed.
    # Eigen's packet padd rounds to f32, so narrow every tree edge.
    p0 = f32(pair0)
    p1 = f32(pair1)
    p2 = f32(pair2)
    p3 = f32(pair3)
    first = f32(p0 + p1)
    second = f32(p2 + p3)
    return FMA(a[8], b[8], f32(first + second))


def gemm_w_j_gebp(w: list[list[float]], j: list[list[float]]) -> list[list[float]]:
    result = [[0.0] * 30 for _ in range(9)]
    for row in range(8):
        for col in range(30):
            result[row][col] = fma_sum(
                [w[row][k] for k in range(9)],
                [j[k][col] for k in range(9)])
    for col in range(30):
        values_a = w[8]
        values_b = [j[k][col] for k in range(9)]
        result[8][col] = (gebp_row8_tree(values_a, values_b)
                          if col < 28 else fma_sum(values_a, values_b))
    return result


def gemm_w_j_gebp_exact(w: list[list[float]], j: list[list[float]]) -> list[list[float]]:
    """Model Eigen 5.0.1 AVX GEBP for the 9x9 * 9x30 boundary.

    The ordinary 1x4 packet micro-kernel keeps even and odd depth terms in
    separate accumulators for columns 0..27, then combines those accumulators
    with packet adds before the remaining depth term is fused.  The final
    scalar row is dispatched through the swapped-tail path: four interleaved
    two-depth packet blocks are accumulated, reduced with the packet-add tree,
    and only then is depth 8 added.  This is deliberately a diagnostic model
    of the pinned Eigen kernel, not a production replacement.
    """

    def one_packet_row(a: list[float], b: list[float]) -> float:
        even = 0.0
        odd = 0.0
        for k in range(0, 8, 2):
            even = FMA(a[k], b[k], even)
        for k in range(1, 8, 2):
            odd = FMA(a[k], b[k], odd)
        return FMA(a[8], b[8], f32(even + odd))

    result = [[0.0] * 30 for _ in range(9)]

    # The full Packet8 rows use the 1x4 GEBP path for the first 28 columns.
    for row in range(8):
        for col in range(28):
            result[row][col] = one_packet_row(
                w[row], [j[k][col] for k in range(9)])
        # The final two columns are the one-column packet remainder and keep
        # the direct depth order.
        for col in range(28, 30):
            result[row][col] = fma_sum(w[row], [j[k][col] for k in range(9)])

    # The scalar row is handled by Eigen's swapped packet tail.  Packets hold
    # [col0-k0, col1-k0, col2-k0, col3-k0, col0-k1, ...].  Each two-depth
    # block has its own accumulator; the four blocks are merged with packet
    # padd, then the two AVX halves are reduced before k=8 is fused.
    for group in range(7):
        columns = [
            [j[k][group * 4 + col] for k in range(9)]
            for col in range(4)
        ]
        blocks: list[list[float]] = []
        for pair in range(4):
            block: list[float] = []
            for k in (2 * pair, 2 * pair + 1):
                for col in range(4):
                    block.append(FMA(columns[col][k], w[8][k], 0.0))
            blocks.append(block)
        merged = [
            f32(f32(blocks[0][lane] + blocks[1][lane]) +
                f32(blocks[2][lane] + blocks[3][lane]))
            for lane in range(8)
        ]
        for col in range(4):
            reduced = f32(merged[col] + merged[col + 4])
            result[8][group * 4 + col] = FMA(
                columns[col][8], w[8][8], reduced)

    for col in range(28, 30):
        result[8][col] = fma_sum(w[8], [j[k][col] for k in range(9)])
    return result


def gemm_w_j_mul_add(w: list[list[float]], j: list[list[float]]) -> list[list[float]]:
    return [[mul_add_sum([w[row][k] for k in range(9)],
                         [j[k][col] for k in range(9)])
             for col in range(30)] for row in range(9)]


def gemm_jt_j_fma(j: list[list[float]]) -> list[list[float]]:
    return [[fma_sum([j[k][left] for k in range(9)],
                      [j[k][right] for k in range(9)])
             for right in range(30)] for left in range(30)]


def gemm_jt_j_mul_add(j: list[list[float]]) -> list[list[float]]:
    return [[mul_add_sum([j[k][left] for k in range(9)],
                         [j[k][right] for k in range(9)])
             for right in range(30)] for left in range(30)]


def gemv_jt_r_tree(j: list[list[float]], r: list[float]) -> list[float]:
    result: list[float] = []
    for col in range(30):
        lanes = [FMA(j[k][col], r[k], 0.0) for k in range(8)]
        q0 = f32(lanes[0] + lanes[4])
        q1 = f32(lanes[1] + lanes[5])
        q2 = f32(lanes[2] + lanes[6])
        q3 = f32(lanes[3] + lanes[7])
        packet_sum = f32(f32(q0 + q2) + f32(q1 + q3))
        result.append(f32(packet_sum + f32(j[8][col] * r[8])))
    return result


def gemv_jt_r_fma(j: list[list[float]], r: list[float]) -> list[float]:
    return [fma_sum([j[k][col] for k in range(9)], r) for col in range(30)]


def gemv_jt_r_mul_add(j: list[list[float]], r: list[float]) -> list[float]:
    return [mul_add_sum([j[k][col] for k in range(9)], r) for col in range(30)]


def sources_from_oracle(oracle: dict[str, Any]) -> tuple[dict[str, Any], dict[str, str]]:
    factor = oracle["frame4_factor"]
    fields = {
        "sqrt_information": "sqrt_information",
        "raw_residual": "raw_residual",
        "jacobian": "jacobian",
    }
    return factor, fields


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--oracle", type=Path, default=DEFAULT_ORACLE)
    parser.add_argument("--rust", type=Path,
                        help="optional injected Rust audit JSON")
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    oracle = json.loads(args.oracle.read_text())
    oracle_factor, oracle_sources = sources_from_oracle(oracle)
    w = read_matrix(oracle_factor["sqrt_information"])
    r = read_array(oracle_factor["raw_residual"])
    j = read_matrix(oracle_factor["jacobian"])
    if (len(w), len(w[0]), len(r), len(j), len(j[0])) != (9, 9, 9, 9, 30):
        raise ValueError("expected W=9x9, r=9, J=9x30")

    expected = {
        "W_r": oracle_field_bits(oracle_factor["whitened_residual"]),
        "W_J": oracle_field_bits(oracle_factor["whitened_jacobian"]),
        "Jt_J": oracle_field_bits(oracle_factor["row_h"]),
        "Jt_r": oracle_field_bits(oracle_factor["row_b"]),
    }

    candidates: dict[str, dict[str, Any]] = {}
    wr_candidates = {
        "gemv_packet_fma": gemv_w_r_packet(w, r),
        "gemv_scalar_fma": gemv_w_r_fma(w, r),
        "gemv_scalar_mul_add": gemv_w_r_mul_add(w, r),
    }
    wj_candidates = {
        "gemm_scalar_fma": gemm_w_j_fma(w, j),
        "gemm_gebp_row8_tree": gemm_w_j_gebp(w, j),
        "gemm_gebp_eigen_avx_exact": gemm_w_j_gebp_exact(w, j),
        "gemm_scalar_mul_add": gemm_w_j_mul_add(w, j),
    }
    # H and b are products of the *whitened* intermediates, not the raw J/r.
    # Keep the producer schedule in each key: this makes a mismatch at W*J
    # distinguishable from a later reduction mismatch.

    def add_vector(name: str, value: list[float], expected_name: str) -> None:
        actual = vector_bits(value)
        candidates[name] = {
            "output": vector_object(value),
            "comparison": compare_words(expected[expected_name], actual),
        }

    def add_matrix(name: str, value: list[list[float]], expected_name: str) -> None:
        actual = matrix_bits(value)
        candidates[name] = {
            "output": matrix_object(value),
            "comparison": compare_words(expected[expected_name], actual),
        }

    for name, value in wr_candidates.items():
        add_vector("W_r/" + name, value, "W_r")
    for name, value in wj_candidates.items():
        add_matrix("W_J/" + name, value, "W_J")
    for wj_name, wj_value in wj_candidates.items():
        for name, value in {
            "gemm_scalar_fma": gemm_jt_j_fma(wj_value),
            "gemm_scalar_mul_add": gemm_jt_j_mul_add(wj_value),
        }.items():
            add_matrix("Jt_J/" + wj_name + "/" + name, value, "Jt_J")
        for wr_name, wr_value in wr_candidates.items():
            for name, value in {
                "gemv_row_packet_tree": gemv_jt_r_tree(wj_value, wr_value),
                "gemv_scalar_fma": gemv_jt_r_fma(wj_value, wr_value),
                "gemv_scalar_mul_add": gemv_jt_r_mul_add(wj_value, wr_value),
            }.items():
                add_vector("Jt_r/" + wr_name + "/" + wj_name + "/" + name,
                           value, "Jt_r")

    rust_result: dict[str, Any] = {"status": "not_requested"}
    if args.rust is not None:
        link, mapping = rust_link_fields(args.rust)
        rust_result = {
            "status": "compared",
            "path": str(args.rust),
            "selection": mapping,
            "fields": {
                name: {
                    "source_field": source,
                    "bits": oracle_field_bits(link[source]),
                }
                for name, source in mapping.items()
            },
        }
        rust_result["comparisons"] = {
            "W_r": compare_words(expected["W_r"], rust_result["fields"]["whitened_residual"]["bits"]),
            "W_J": compare_words(expected["W_J"], rust_result["fields"]["whitened_jacobian"]["bits"]),
            "Jt_J": compare_words(expected["Jt_J"], rust_result["fields"]["row_h"]["bits"]),
            "Jt_r": compare_words(expected["Jt_r"], rust_result["fields"]["row_b"]["bits"]),
        }

    output = {
        "schema": "basalt.m7im_whitening_schedule_fixture.v1",
        "status": "diagnostic_only",
        "oracle": str(args.oracle),
        "oracle_schema": oracle.get("schema"),
        "source_contract": {
            "pinned_upstream_commit": "0f3b2b52c807f70ff4e2973ce253c73329eea7bc",
            "oracle_factor": "frame4_factor",
            "scalar": "float32",
            "arithmetic": "platform C fmaf for FMA candidates; explicit f32 narrowing for add/tree edges",
            "fmaf_library": FMA_LIBRARY,
            "schedule_report": "benchmarks/basalt/m7im_whitening_schedule_report_20260824.md",
            "production_source_touched": False,
            "legacy_widened_fields_used": False,
        },
        "inputs": {
            name: {
                "source_field": source,
                "shape": ([9, 9] if name == "sqrt_information" else
                          [9, 30] if name == "jacobian" else [9]),
                "bits": (matrix_bits(read_matrix(oracle_factor[source]))
                         if name != "raw_residual"
                         else vector_bits(read_array(oracle_factor[source]))),
            }
            for name, source in oracle_sources.items()
        },
        "expected_outputs": expected,
        "candidates": candidates,
        "rust": rust_result,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(output, indent=2) + "\n")

    exact = [
        (name, value["comparison"])
        for name, value in candidates.items()
        if value["comparison"]["exact"]
    ]
    print(json.dumps({
        "report": str(args.report),
        "fmaf_library": FMA_LIBRARY,
        "candidate_count": len(candidates),
        "exact_candidates": [name for name, _ in exact],
        "rust": rust_result.get("comparisons"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
