"""Compare the clean GDB sqrt_cov_inv capture with Rust/native audit JSON."""

from __future__ import annotations

import json
import re
import struct
from pathlib import Path


def bits(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", value))[0]


def capture(path: Path) -> list[list[int]]:
    chunks: list[list[int]] = []
    current: list[int] | None = None
    for line in path.read_text().splitlines():
        if "M7HD sqrt_cov_inv 81 float words" in line:
            current = []
            chunks.append(current)
        elif current is not None and line.startswith("0x") and ":" in line:
            current.extend(int(word, 16) for word in line.split(":", 1)[1].split())
    if any(len(chunk) != 81 for chunk in chunks):
        raise RuntimeError([len(chunk) for chunk in chunks])
    return chunks


def main() -> None:
    native = capture(Path("target/m7hd_clean_sqrt_capture.out"))
    for name in ("m7aq_upstream_imu_audit.json", "m7aq_rust_imu_audit.json"):
        audit = json.loads(Path("target", name).read_text())
        print(name)
        for link, matrix in enumerate(audit["links"]):
            rust = [bits(value) for row in matrix["sqrt_information"] for value in row]
            # Eigen's Matrix memory is column-major; JSON matrices are row-major.
            native_row_major = [native[link][row + 9 * col] for row in range(9) for col in range(9)]
            mismatch = [index for index, (lhs, rhs) in enumerate(zip(native_row_major, rust)) if lhs != rhs]
            print(link, "bits", len(mismatch), "first", mismatch[:8])
            for index in mismatch[:4]:
                print(" ", index, hex(native_row_major[index]), hex(rust[index]))


if __name__ == "__main__":
    main()
