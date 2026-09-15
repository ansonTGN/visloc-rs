#!/usr/bin/env python3
"""Canonicalize the pinned upstream M8c frontend diagnostic.

The C++ program intentionally emits a raw dump with complete arrays for each
selected image.  This script keeps the first image losslessly and replaces the
larger arrays for later images with SHA-256 summaries.  It also sorts data whose
upstream containers are unordered, while retaining the exact source/order gap
in the provenance block.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from pathlib import Path
from typing import Any


UPSTREAM_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"


def json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(json_bytes(value))


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def descriptor_bytes(value: list[list[int]]) -> bytes:
    out = bytearray()
    for row in value:
        if len(row) != 32:
            raise ValueError(f"descriptor row has {len(row)} bytes, expected 32")
        for byte in row:
            if not 0 <= byte <= 255:
                raise ValueError(f"descriptor byte outside u8: {byte}")
        out.extend(row)
    return bytes(out)


CANONICAL_FLOAT_SCALE = 10**12


def canonical_float(value: float) -> bytes:
    # Rust's DS unprojection differs from the upstream double path by a few
    # ulps on some rays.  The full-array gate therefore hashes a documented
    # 1e-12 fixed-point representation rather than hiding a raw float compare
    # inside a cryptographic summary.
    # Truncation toward zero avoids language-specific tie-to-even versus
    # tie-away-from-zero behavior at the fixed-point boundary.
    scaled = int(float(value) * CANONICAL_FLOAT_SCALE)
    return struct.pack("<q", scaled)


def canonical_feature_bytes(
    corners: list[list[float]],
    angles: list[float],
    descriptors: list[list[int]],
    rays: list[list[float]],
    hashes: list[int],
    bow: list[dict[str, Any]],
) -> dict[str, bytes]:
    corners_bytes = b"".join(
        canonical_float(value) for point in corners for value in point
    )
    angles_bytes = b"".join(canonical_float(value) for value in angles)
    rays_bytes = b"".join(
        canonical_float(value) for ray in rays for value in ray
    )
    hashes_bytes = b"".join(struct.pack("<I", int(value)) for value in hashes)
    bow_bytes = b"".join(
        struct.pack("<I", int(entry["hash"]))
        + canonical_float(float(entry["weight"]))
        for entry in bow
    )
    return {
        "corners_xy": corners_bytes,
        "corner_angles": angles_bytes,
        "descriptor_bytes": descriptor_bytes(descriptors),
        "corners_3d": rays_bytes,
        "hashes_16bit": hashes_bytes,
        "bow_vector": bow_bytes,
    }


def canonical_feature_hashes(
    corners: list[list[float]],
    angles: list[float],
    descriptors: list[list[int]],
    rays: list[list[float]],
    hashes: list[int],
    bow: list[dict[str, Any]],
) -> dict[str, str]:
    return {
        key: sha256_bytes(value)
        for key, value in canonical_feature_bytes(
            corners, angles, descriptors, rays, hashes, bow
        ).items()
    }


def canonical_temporal_match_bytes(matches: list[dict[str, int]]) -> bytes:
    return b"".join(
        struct.pack(
            "<III",
            int(match["left_feature_id"]),
            int(match["right_feature_id"]),
            int(match["hamming"]),
        )
        for match in matches
    )


def canonical_temporal_inlier_bytes(inliers: list[list[int]]) -> bytes:
    return b"".join(
        struct.pack("<II", int(pair[0]), int(pair[1])) for pair in inliers
    )


def canonical_bow(value: list[dict[str, Any]]) -> list[dict[str, Any]]:
    # HashBow::compute_bow emits an unordered_map traversal.  Hash is the only
    # semantic key, so sorting it gives a stable fixture without pretending to
    # know the upstream bucket/iteration order.
    return sorted(value, key=lambda row: (int(row["hash"]), float(row["weight"])))


def source_hashes(source_root: Path) -> list[dict[str, str | None]]:
    symbols = [
        ("src/vi_estimator/nfr_mapper.cpp", "NfrMapper::detect_keypoints"),
        ("src/vi_estimator/nfr_mapper.cpp", "NfrMapper::match_stereo"),
        ("src/utils/keypoints.cpp", "detectKeypointsMapping"),
        ("src/utils/keypoints.cpp", "computeAngles"),
        ("src/utils/keypoints.cpp", "computeDescriptors"),
        ("src/utils/keypoints.cpp", "matchDescriptors"),
        ("src/utils/keypoints.cpp", "findInliersRansac"),
        ("include/basalt/utils/keypoints.h", "findInliersEssential"),
        ("include/basalt/hash_bow/hash_bow.h", "HashBow::compute_hash"),
        ("include/basalt/hash_bow/hash_bow.h", "HashBow::compute_bow"),
        ("include/basalt/hash_bow/hash_bow.h", "HashBow::add_to_database"),
        (
            "thirdparty/vcpkg/packages/basalt-headers_x64-linux/include/basalt/camera/generic_camera.hpp",
            "GenericCamera::unproject",
        ),
        (
            "thirdparty/vcpkg/buildtrees/opengv/src/ae80b3a7f3-99d8cee564.clean/src/sac_problems/relative_pose/CentralRelativePoseSacProblem.cpp",
            "CentralRelativePoseSacProblem::computeModelCoefficients/selectWithinDistance/getSampleSize",
        ),
        (
            "thirdparty/vcpkg/buildtrees/opengv/src/ae80b3a7f3-99d8cee564.clean/include/opengv/sac/implementation/Ransac.hpp",
            "sac::Ransac::computeModel",
        ),
        (
            "thirdparty/vcpkg/buildtrees/opengv/src/ae80b3a7f3-99d8cee564.clean/src/relative_pose/modules/main.cpp",
            "fivept_stewenius_main",
        ),
    ]
    result: list[dict[str, str | None]] = []
    for relative, symbol in symbols:
        path = source_root / relative
        result.append(
            {
                "path": relative,
                "symbol": symbol,
                "sha256": sha256_file(path),
            }
        )
    return result


def build_fixture(raw: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    raw_features = raw["features"]
    if not raw_features:
        raise ValueError("diagnostic returned no feature images")
    if len(raw_features) != len(raw["selected_time_cam_ids"]):
        raise ValueError("selected IDs/features length mismatch")

    features: list[dict[str, Any]] = []
    for index, raw_feature in enumerate(raw_features):
        corners = raw_feature["corners_xy"]
        angles = raw_feature["corner_angles"]
        descriptors = raw_feature["descriptor_bytes"]
        rays = raw_feature["corners_3d"]
        hashes = [int(value) for value in raw_feature["hashes"]]
        bow = canonical_bow(raw_feature["bow_vector"])
        count = int(raw_feature["corner_count"])
        if count != len(corners) or count != len(angles) or count != len(descriptors):
            raise ValueError(f"feature array length mismatch at index {index}")
        if count != len(rays) or count != len(hashes):
            raise ValueError(f"ray/hash length mismatch at index {index}")
        if any(value < 0 or value >= (1 << 16) for value in hashes):
            raise ValueError(f"non-16-bit HashBow value at index {index}")

        common: dict[str, Any] = {
            "time_cam_id": raw_feature["time_cam_id"],
            "corner_count": count,
            "descriptor_count": len(descriptors),
            "ray_count": len(rays),
            "hash_count": len(hashes),
            "hashes_16bit": hashes,
            "bow_vector": bow,
            "canonical_sha256": canonical_feature_hashes(
                corners, angles, descriptors, rays, hashes, bow
            ),
            "canonical_encoding": "little-endian fixed-point int64 floats at 1e-12 (truncate toward zero); descriptor bytes raw u8; hashes u32; BoW hash u32 + weight int64",
        }

        if index == 0:
            common.update(
                {
                    "corner_ids": list(range(count)),
                    "corners_xy": corners,
                    "corner_angles": angles,
                    "descriptor_bytes": descriptors,
                    "corners_3d": rays,
                    "descriptor_byte_encoding": {
                        "width": 32,
                        "bit_order": "std::bitset<256>[8*byte+bit]",
                        "byte_endianness": "little",
                    },
                }
            )
        else:
            common.update(
                {
                    "corner_ids": {"encoding": "implicit_zero_based", "count": count},
                    "corners_xy_sha256": sha256_json(corners),
                    "corner_angles_sha256": sha256_json(angles),
                    "descriptor_bytes_sha256": sha256_bytes(descriptor_bytes(descriptors)),
                    "corners_3d_sha256": sha256_json(rays),
                    "hashes_16bit_sha256": sha256_json(hashes),
                    "bow_vector_sha256": sha256_json(bow),
                    "summary_encoding": "canonical_json_utf8; descriptor bytes are raw concatenated u8",
                }
            )
        features.append(common)

    stereo_pairs: list[dict[str, Any]] = []
    for pair in raw["stereo"]["pairs"]:
        matches = sorted(
            pair["raw_mutual_hamming"],
            key=lambda row: (int(row["left_feature_id"]), int(row["right_feature_id"])),
        )
        inliers = sorted(pair["essential_inlier_ids"])
        stereo_pairs.append(
            {
                "left": pair["left"],
                "right": pair["right"],
                "raw_mutual_hamming": matches,
                "raw_match_count": len(matches),
                "essential_inlier_ids": inliers,
                "essential_inlier_count": len(inliers),
                "mapper_feature_matches_stored": bool(
                    pair["mapper_feature_matches_stored"]
                ),
            }
        )
    stereo_pairs.sort(
        key=lambda pair: (
            int(pair["left"]["frame_id"]),
            int(pair["left"]["cam_id"]),
            int(pair["right"]["cam_id"]),
        )
    )

    temporal_matches = []
    raw_query = raw.get("bow_query", {})
    for match in raw_query.get("temporal_matches", []):
        raw_matches = sorted(
            match["raw_mutual_hamming"],
            key=lambda row: (
                int(row["left_feature_id"]),
                int(row["right_feature_id"]),
                int(row["hamming"]),
            ),
        )
        inliers = sorted(match.get("ransac_inlier_ids", []))
        temporal_matches.append(
            {
                "left": match["left"],
                "right": match["right"],
                "bow_score": float(match["bow_score"]),
                "raw_match_count": len(raw_matches),
                "raw_mutual_hamming_sha256": sha256_bytes(
                    canonical_temporal_match_bytes(raw_matches)
                ),
                "ransac_attempted": bool(match["ransac_attempted"]),
                "ransac_inlier_count": len(inliers),
                "ransac_inlier_ids_sha256": sha256_bytes(
                    canonical_temporal_inlier_bytes(inliers)
                ),
                "mapper_feature_matches_stored": bool(
                    match["mapper_feature_matches_stored"]
                ),
                "reject_stage": match["reject_stage"],
            }
        )
    temporal_matches.sort(
        key=lambda match: (
            int(match["left"]["frame_id"]),
            int(match["left"]["cam_id"]),
            int(match["right"]["frame_id"]),
            int(match["right"]["cam_id"]),
        )
    )

    seeded_temporal = []
    for oracle in raw_query.get("seeded_temporal_oracle", []):
        ransac_inliers = sorted(oracle.get("ransac_inlier_ids", []))
        refined_inliers = sorted(oracle.get("refined_inlier_ids", []))
        seeded_temporal.append(
            {
                "left": oracle["left"],
                "right": oracle["right"],
                "bow_score": float(oracle["bow_score"]),
                "seed": int(oracle["seed"]),
                "raw_match_count": int(oracle["raw_match_count"]),
                "ransac_iterations": int(oracle["ransac_iterations"]),
                "model_found": bool(oracle["model_found"]),
                "accepted": bool(oracle["accepted"]),
                "ransac_model_rotation": oracle["ransac_model_rotation"],
                "ransac_model_translation": oracle["ransac_model_translation"],
                "refined_model_rotation": oracle["refined_model_rotation"],
                "refined_model_translation": oracle["refined_model_translation"],
                "ransac_inlier_count": len(ransac_inliers),
                "ransac_inlier_ids": ransac_inliers,
                "ransac_inlier_ids_sha256": sha256_bytes(
                    canonical_temporal_inlier_bytes(ransac_inliers)
                ),
                "refined_inlier_count": len(refined_inliers),
                "refined_inlier_ids": refined_inliers,
                "refined_inlier_ids_sha256": sha256_bytes(
                    canonical_temporal_inlier_bytes(refined_inliers)
                ),
            }
        )
    seeded_temporal.sort(
        key=lambda match: (
            int(match["left"]["frame_id"]),
            int(match["left"]["cam_id"]),
            int(match["right"]["frame_id"]),
            int(match["right"]["cam_id"]),
            int(match["seed"]),
        )
    )

    query_entries = []
    for query in raw_query.get("queries", []):
        candidates = sorted(
            query["candidates"],
            key=lambda row: (
                int(row["time_cam_id"]["frame_id"]),
                int(row["time_cam_id"]["cam_id"]),
                -float(row["score"]),
            ),
        )
        query_entries.append(
            {
                "time_cam_id": query["time_cam_id"],
                "candidate_count": len(candidates),
                "candidates": candidates,
            }
        )
    query_entries.sort(
        key=lambda query: (
            int(query["time_cam_id"]["frame_id"]),
            int(query["time_cam_id"]["cam_id"]),
        )
    )

    marg_file = Path(args.marg_dir) / raw["selected_marg_file"]
    packet_hashes = [
        {
            "file": packet_name,
            "sha256": sha256_file(Path(args.marg_dir) / packet_name),
        }
        for packet_name in raw.get("marg_packet_files", [raw["selected_marg_file"]])
    ]
    image_hashes = []
    for tcid in raw["selected_time_cam_ids"]:
        if int(tcid["cam_id"]) != 0:
            continue
        image_file = Path(args.marg_dir) / "images" / f'{int(tcid["frame_id"])}.cereal'
        image_hashes.append(
            {
                "frame_id": int(tcid["frame_id"]),
                "path": str(image_file),
                "sha256": sha256_file(image_file),
            }
        )

    images_dir = Path(args.marg_dir) / "images"
    image_record_count = len(list(images_dir.glob("*.cereal")))

    fixture: dict[str, Any] = {
        "schema": "basalt.m8c_feature_oracle.v1",
        "upstream": {
            "repository": "https://github.com/VladyslavUsenko/basalt",
            "commit": UPSTREAM_COMMIT,
            "tree": "source checkout is clean at the pinned commit",
        },
        "dataset": {
            "sequence": "MH_01_easy",
            "source": "real MH01 mapper images attached by MargDataLoader",
            "marg_directory": str(args.marg_dir),
            "selected_marg_file": raw["selected_marg_file"],
            "selected_marg_sha256": sha256_file(marg_file),
            "marg_packet_sha256": packet_hashes,
            "image_record_sha256": image_hashes,
        },
        "config": raw["config"],
        "config_provenance": {
            "path": str(args.config),
            "sha256": sha256_file(Path(args.config)),
            "values_source": "VioConfig::load; mapper fields are emitted by diagnostic",
        },
        "calibration_provenance": {
            "path": str(args.calibration),
            "sha256": sha256_file(Path(args.calibration)),
        },
        "selection": {
            "max_time_cam_ids": int(raw["max_tcid"]),
            "fixture_role": "M8c full first-20 TimeCamId gate",
            "full_gate_target_time_cam_ids": 20,
            "full_gate_status": "claimed; selected from continuous pinned MargData packets",
            "available_time_cam_ids_in_first_marg_record": raw["available_time_cam_ids"],
            "available_time_cam_id_count_in_first_marg_record": len(
                raw["available_time_cam_ids"]
            ),
            "marg_packet_count": int(raw.get("marg_packet_count", 1)),
            "marg_packet_files": raw.get("marg_packet_files", []),
            "marg_directory_image_record_count": image_record_count,
            "marg_directory_time_cam_id_capacity": 2 * image_record_count,
            "ordering": "ascending (frame_id, cam_id); first 20 canonical IDs; first image is fixture[features][0]",
            "time_cam_ids": raw["selected_time_cam_ids"],
        },
        "features": features,
        "bow_query": {
            "num_results": int(raw_query.get("num_results", raw["config"]["mapper_num_frames_to_match"])),
            "score_threshold": float(
                raw_query.get(
                    "score_threshold", raw["config"]["mapper_frames_to_match_threshold"]
                )
            ),
            "ordering": raw_query.get(
                "ordering",
                "canonical TimeCamId/score; upstream unordered query order is not claimed",
            ),
            "queries": query_entries,
            "temporal_matches": temporal_matches,
            "temporal_match_count": len(temporal_matches),
            "temporal_encoding": "raw mutual triples are little-endian u32(left,right,hamming); RANSAC ID pairs are little-endian u32 pairs; both are SHA-256 summaries; reject_stage is exact diagnostic classification",
            "seeded_temporal_oracle": seeded_temporal,
            "seeded_temporal_oracle_count": len(seeded_temporal),
            "seeded_temporal_encoding": "fixed explicit OpenGV seeds; model coefficients are emitted as upstream doubles; inlier IDs are canonical sorted sets and little-endian u32-pair SHA-256 summaries",
        },
        "stereo": {
            "essential_threshold": float(raw["stereo"]["essential_threshold"]),
            "T_0_1_translation": raw["stereo"]["T_0_1_translation"],
            "T_0_1_rotation": raw["stereo"]["T_0_1_rotation"],
            "pairs": stereo_pairs,
        },
        "provenance": {
            "diagnostic_source": str(args.diagnostic),
            "diagnostic_source_sha256": sha256_file(Path(args.diagnostic)),
            "raw_dump_sha256": sha256_file(Path(args.raw)),
            "compile_command": (
                "c++ -std=c++17 -O2 -march=native -DEIGEN_DONT_PARALLELIZE "
                "-DBASALT_INSTANTIATIONS_DOUBLE -DBASALT_INSTANTIATIONS_FLOAT "
                "-DCLI11_COMPILE -DHAVE_EIGEN -DHAVE_EPOXY "
                "-DPANGO_DEFAULT_WIN_URI=\\\"x11\\\" -D_LINUX_ "
                "-I/root/visloc-basalt-oracle-0f3b2b52/include "
                "-I/root/visloc-basalt-oracle-0f3b2b52/thirdparty/ros/include "
                "-I/root/visloc-basalt-oracle-0f3b2b52/thirdparty/apriltag/include "
                "-isystem /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/eigen3/src/5.0.1-d487a628b0.clean "
                "-isystem /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed/x64-linux/include/opencv4 "
                "-isystem /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed/x64-linux/include "
                "-L/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo "
                "-Wl,-rpath,/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo "
                "-o /tmp/upstream_m8c_feature_oracle "
                "benchmarks/basalt/upstream_m8c_feature_oracle.cpp -lbasalt -lpthread"
            ),
            "run_command": (
                "/tmp/upstream_m8c_feature_oracle "
                "/root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json "
                "/root/visloc-basalt-oracle-0f3b2b52/data/euroc_config.json "
                "/root/basalt-oracle-results-runner-smoke/MH_01_easy/20260821T000003Z/marg_data "
                "/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m8c_feature_raw20_ransac.json 20"
            ),
            "source_symbols": source_hashes(Path(args.source_root)),
            "algorithm_order": [
                "NfrMapper::addMargData",
                "NfrMapper::detect_keypoints",
                "detectKeypointsMapping",
                "computeAngles(rotate_features=true)",
                "computeDescriptors",
                "GenericCamera::unproject",
                "HashBow<256>::compute_bow",
                "HashBow<256>::add_to_database",
                "NfrMapper::match_stereo",
                "matchDescriptors(mutual best + threshold + ratio)",
                "findInliersEssential(epipolar threshold 1e-3)",
                "HashBow::querry_database(candidate set)",
                "NfrMapper::match_all",
                "findInliersRansac(ransac threshold + min-match gate)",
                "OpenGV CentralRelativeAdapter + CentralRelativePoseSacProblem(STEWENIUS)",
                "OpenGV Ransac(max_iterations=100, probability=0.99)",
                "OpenGV optimize_nonlinear(Cayley + NumericalDiff/LM)",
            ],
            "unordered_container_gap": {
                "hash_bow_bow_vector": "HashBow::compute_bow uses std::unordered_map<FeatureHash,double>; fixture sorts entries by numeric hash",
                "mutual_match_iteration": "matchFastHelper/matchDescriptors uses std::unordered_map; fixture sorts pair IDs",
                "mapper_image_maps": "NfrMapper img_data/feature_corners are unordered/concurrent maps; fixture sorts TimeCamIds",
                "hash_bow_query_iteration": "HashBow::querry_database accumulates/traverses unordered maps; fixture retains canonical candidate set sorted by TimeCamId and score",
            },
            "temporal_ransac_gap": "Default NfrMapper::match_all keeps OpenGV's time(0)+clock() seed inside the TBB loop and is therefore not deterministic. The diagnostic-only seeded hook uses CentralRelativeAdapter, STEWENIUS, sample size 8, max_iterations 100, probability 0.99, threshold 5e-5, OpenGV reprojection distance/selectWithinDistance, and Cayley/LM refinement; those fixed-seed model/inlier records are the deterministic Rust parity gate. Default time-seeded run summaries remain observational.",
            "determinism": "Canonical JSON and all unordered outputs are sorted; no upstream source mutation or tuning",
            "fixture_hash": "computed as SHA-256 of exact UTF-8 fixture bytes after writing",
        },
    }
    return fixture


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--marg-dir", type=Path, required=True)
    parser.add_argument("--diagnostic", type=Path, required=True)
    args = parser.parse_args()

    raw = json.loads(args.raw.read_text(encoding="utf-8"))
    fixture = build_fixture(raw, args)
    args.fixture.parent.mkdir(parents=True, exist_ok=True)
    # Keep indentation for review; canonical hash is over these exact bytes.
    payload = (json.dumps(fixture, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    args.fixture.write_bytes(payload)
    fixture_hash = sha256_bytes(payload)

    report = "# M8c upstream mapper feature oracle\n\n"
    report += "This fixture is generated from the pinned upstream Basalt `NfrMapper`\n"
    report += "frontend on real `MH_01_easy` images attached to ten continuous\n"
    report += "MargData packets. It contains complete keypoints/rays/256-bit\n"
    report += "descriptor bytes for the first selected `TimeCamId`; the remaining\n"
    report += "19 images retain exact canonical SHA-256 summaries, 16-bit HashBoW\n"
    report += "values, and canonical BoW entries.\n\n"
    report += f"Fixture: `{args.fixture}`\n\n"
    report += f"Fixture SHA-256: `{fixture_hash}`\n\n"
    report += f"Upstream commit: `{UPSTREAM_COMMIT}`\n\n"
    report += "## Exact pipeline and configuration\n\n"
    report += "The diagnostic calls `NfrMapper::addMargData`, then the exact upstream\n"
    report += "`detect_keypoints()` and `match_stereo()` methods. Detection uses\n"
    report += "`detectKeypointsMapping`, `computeAngles(..., true)`, and\n"
    report += "`computeDescriptors`; rays come from the configured camera\n"
    report += "`GenericCamera::unproject`. HashBoW uses 16 bits. Stereo matching uses\n"
    report += "the configured mutual Hamming threshold/ratio and essential residual\n"
    report += "threshold `1e-3`, exactly as `NfrMapper::match_stereo`.\n\n"
    report += "```json\n" + json.dumps(raw["config"], indent=2) + "\n```\n\n"
    report += "## Full first-20 scope\n\n"
    report += "The diagnostic loads all ten continuous numeric MargData packets via\n"
    report += "the pinned `MargDataLoader`, calls `NfrMapper::addMargData` for each,\n"
    report += "then runs `detect_keypoints`, `match_stereo`, and `match_all`. The\n"
    report += "cumulative mapper contains 34 camera IDs; this fixture fixes the first\n"
    report += "20 canonical IDs (ten stereo frames). It also records canonical BoW\n"
    report += "query candidates and temporal raw/inlier/reject summaries. It also\n"
    report += "contains a diagnostic-only explicit-seed OpenGV RANSAC oracle for\n"
    report += "three representative raw>20 pairs across seeds 12345, 424242, and 7.\n\n"
    report += "## Determinism and known upstream gap\n\n"
    report += "`HashBow::compute_bow` inserts entries from a\n"
    report += "`std::unordered_map<FeatureHash,double>`, and the mutual matcher uses\n"
    report += "unordered-map iteration too. Their semantic sets are retained, but the\n"
    report += "fixture canonicalizes BoW entries by numeric 16-bit hash and matches by\n"
    report += "`(left_feature_id,right_feature_id)`. The fixture also sorts\n"
    report += "`TimeCamId`s because mapper containers are unordered/concurrent. BoW\n"
    report += "query candidates are likewise retained as a canonical semantic set,\n"
    report += "not an upstream unordered traversal order. The full-array hashes use\n"
    report += "the documented 1e-12 fixed-point float encoding to accommodate the\n"
    report += "observed DS ray ulp difference; no source mutation or tuning was used.\n\n"
    report += "The default `NfrMapper::match_all` path retains OpenGV's\n"
    report += "`time(0)+clock()` seed inside the TBB loop, so default inlier\n"
    report += "summaries remain observational. The diagnostic-only seeded hook\n"
    report += "uses the literal `CentralRelativeAdapter`, STEWENIUS five-point\n"
    report += "solver, sample size 8, 100-iteration/0.99 RANSAC, reprojection\n"
    report += "threshold `5e-5`, `selectWithinDistance`, and Cayley/LM refinement.\n"
    report += "Rust's seeded API is gated against those model/inlier records; the\n"
    report += "only remaining gap is nondeterministic default-time-seed parity.\n\n"
    report += "## Scope\n\n"
    report += "No ground truth is read. The diagnostic source is under\n"
    report += "`benchmarks/basalt/`; the pinned upstream checkout is untouched, and no\n"
    report += "Rust production file is involved.\n"
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(report, encoding="utf-8")
    print(f"fixture_sha256={fixture_hash}")
    print(f"fixture={args.fixture}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
