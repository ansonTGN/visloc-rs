#!/usr/bin/env python3
"""Build the portable M8d track oracle from the pinned upstream matches.

The M8c first-20 gate contains ten exact stereo pairs (twenty TimeCamIds),
while the diagnostic raw dump contains the exact accepted temporal inlier
pairs produced by ``NfrMapper::match_all``.  Both sets are already in the
``MatchData.inliers`` format consumed by upstream ``TrackBuilder``.  This
helper mirrors ``tracks.h`` and ``union_find.h`` only for fixture generation;
the production implementation and the assertions live in ``visloc-basalt``.

The temporal dump is deliberately required for a useful oracle.  Stereo-only
matches are ten disconnected two-image components and therefore all fail the
configured five-image minimum, which would exercise only the short-track
branch and could never validate exported tracks.  The checked-in M8c fixture
stores temporal inlier IDs as hashes, so the exact diagnostic raw dump is
accepted separately and its SHA-256 is recorded in the generated artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from pathlib import Path


INVALID = (1 << 32) - 1
UPSTREAM_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"


class UnionFind:
    def __init__(self, n: int) -> None:
        self.parent = list(range(n))
        self.rank = [0] * n
        self.size = [1] * n

    def find(self, i: int) -> int:
        p = self.parent[i]
        if p != i and p != INVALID:
            self.parent[i] = self.find(p)
        return self.parent[i]

    def union(self, i: int, j: int) -> None:
        i = self.find(i)
        j = self.find(j)
        if i == j:
            return
        if self.rank[i] < self.rank[j]:
            self.parent[i] = j
            self.size[j] += self.size[i]
        else:
            self.parent[j] = i
            self.size[i] += self.size[j]
            if self.rank[i] == self.rank[j]:
                self.rank[i] += 1


def tcid(value: dict[str, int]) -> tuple[int, int]:
    return int(value["frame_id"]), int(value["cam_id"])


def parse_inliers(pair: dict) -> list[tuple[int, int]]:
    result = []
    for value in pair["essential_inlier_ids"]:
        if isinstance(value, str):
            left, right = value.split()
            result.append((int(left), int(right)))
        else:
            result.append((int(value[0]), int(value[1])))
    return result


def parse_temporal_inliers(pair: dict) -> list[tuple[int, int]]:
    """Read exact upstream temporal ``MatchData.inliers`` from a raw dump."""

    values = pair.get("ransac_inlier_ids")
    if values is None:
        raise ValueError(
            "temporal source has no exact ransac_inlier_ids; "
            "the summary-only M8c fixture cannot build tracks"
        )
    return [(int(value[0]), int(value[1])) for value in values]


def match_input_bytes(
    pairs: list[tuple[tuple[int, int], tuple[int, int], list[tuple[int, int]]]],
) -> bytes:
    """Canonical bytes for the exact pair/inlier input used by the oracle."""

    encoded = bytearray()
    for left, right, inliers in sorted(pairs):
        encoded.extend(struct.pack("<QQQQQ", left[0], left[1], right[0], right[1], len(inliers)))
        for feature_left, feature_right in sorted(inliers):
            encoded.extend(struct.pack("<QQ", feature_left, feature_right))
    return bytes(encoded)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def load_temporal_pairs(
    source: dict,
    temporal_source: dict | None,
) -> tuple[list[tuple[tuple[int, int], tuple[int, int], list[tuple[int, int]]]], int]:
    """Return exact accepted temporal pairs and their raw pair count.

    A future M8c fixture may carry exact IDs directly.  For the current
    summary-only fixture, use the separately captured diagnostic dump.
    """

    embedded = source.get("bow_query", {}).get("temporal_matches", [])
    if embedded and "ransac_inlier_ids" in embedded[0]:
        values = embedded
    elif temporal_source is not None:
        values = temporal_source.get("bow_query", {}).get("temporal_matches", [])
    else:
        raise ValueError(
            "exact temporal inlier source is required; pass --temporal-source "
            "using the raw upstream M8c diagnostic dump"
        )

    selected_ids = {
        tcid(value)
        for value in source.get("selection", {}).get("time_cam_ids", [])
    }
    pairs = []
    seen_pairs = set()
    for value in values:
        if value.get("mapper_feature_matches_stored") is False:
            continue
        if value.get("reject_stage") not in (None, "accepted"):
            continue
        left = tcid(value["left"])
        right = tcid(value["right"])
        if selected_ids and (left not in selected_ids or right not in selected_ids):
            raise ValueError(
                "temporal source contains a pair outside the first-20 "
                "TimeCamId selection"
            )
        inliers = sorted(parse_temporal_inliers(value))
        if not inliers:
            continue
        pair_key = (left, right)
        if pair_key in seen_pairs:
            raise ValueError(f"temporal source repeats pair {pair_key}")
        seen_pairs.add(pair_key)
        pairs.append((left, right, inliers))
    if not pairs:
        raise ValueError("temporal source contains no accepted exact inlier pairs")
    return pairs, len(values)


def build(
    source: dict,
    minimum_track_length: int,
    temporal_source: dict | None = None,
) -> tuple[dict, list[dict], list[dict], str]:
    pairs = []
    stereo_pair_count = 0
    all_features: set[tuple[tuple[int, int], int]] = set()
    for pair in source["stereo"]["pairs"]:
        left = tcid(pair["left"])
        right = tcid(pair["right"])
        inliers = sorted(parse_inliers(pair))
        pairs.append((left, right, inliers))
        stereo_pair_count += 1
        for feature_left, feature_right in inliers:
            all_features.add((left, feature_left))
            all_features.add((right, feature_right))

    temporal_pairs, temporal_pair_count = load_temporal_pairs(
        source, temporal_source
    )
    for left, right, inliers in temporal_pairs:
        pairs.append((left, right, inliers))
        for feature_left, feature_right in inliers:
            all_features.add((left, feature_left))
            all_features.add((right, feature_right))

    nodes = sorted(all_features)
    index = {node: i for i, node in enumerate(nodes)}
    uf = UnionFind(len(nodes))
    for left, right, inliers in sorted(pairs):
        for feature_left, feature_right in inliers:
            uf.union(index[(left, feature_left)], index[(right, feature_right)])

    # This is the intentionally slightly unusual upstream Filter traversal:
    # once a root is marked problematic, later nodes in that root are skipped.
    tracks: dict[int, set[tuple[int, int]]] = {}
    conflict: set[int] = set()
    for node, node_index in zip(nodes, range(len(nodes))):
        root = uf.find(node_index)
        if root in conflict:
            continue
        image = node[0]
        images = tracks.setdefault(root, set())
        if image in images:
            conflict.add(root)
        else:
            images.add(image)

    short = {root for root, images in tracks.items() if len(images) < minimum_track_length}
    rejected = conflict | short
    for i, parent in enumerate(uf.parent):
        if parent in rejected:
            uf.size[parent] = 1
            uf.parent[i] = INVALID

    exported: dict[int, list[tuple[tuple[int, int], int]]] = {}
    for node, node_index in zip(nodes, range(len(nodes))):
        root = uf.find(node_index)
        if root != INVALID:
            exported.setdefault(root, []).append((node[0], node[1]))

    histogram: dict[str, int] = {}
    for observations in exported.values():
        key = str(len(observations))
        histogram[key] = histogram.get(key, 0) + 1
    histogram = dict(sorted(histogram.items(), key=lambda item: int(item[0])))

    canonical = bytearray()
    canonical_observations = []
    for root, observations in sorted(exported.items()):
        canonical.extend(struct.pack("<QQ", root, len(observations)))
        canonical_observations.append(
            {
                "id": root,
                "observations": [
                    {
                        "image": {"frame_id": image[0], "cam_id": image[1]},
                        "feature_id": feature,
                    }
                    for image, feature in observations
                ],
            }
        )
        for image, feature in observations:
            canonical.extend(struct.pack("<QQQ", image[0], image[1], feature))

    digest = 1469598103934665603
    for byte in canonical:
        digest = ((digest ^ byte) * 1099511628211) & ((1 << 64) - 1)

    input_hash = sha256_bytes(match_input_bytes(pairs))
    summary = {
        "node_count": len(nodes),
        "component_count_before": len(tracks),
        "component_count_after": len(exported),
        "rejected_conflict_ids": sorted(conflict),
        "rejected_short_ids": sorted(short),
        "rejected_track_ids": sorted(rejected),
        "exported_track_count": len(exported),
        "track_length_histogram": histogram,
        "canonical_observations": canonical_observations,
        "canonical_hash": digest,
        "input_match_pair_count": len(pairs),
        "input_stereo_pair_count": stereo_pair_count,
        "input_temporal_pair_count": len(temporal_pairs),
        "input_temporal_source_pair_count": temporal_pair_count,
        "input_match_sha256": input_hash,
    }
    input_matches = [
        {
            "left": {"frame_id": left[0], "cam_id": left[1]},
            "right": {"frame_id": right[0], "cam_id": right[1]},
            "inliers": [[feature_left, feature_right] for feature_left, feature_right in inliers],
        }
        for left, right, inliers in sorted(pairs)
    ]
    return summary, canonical_observations, input_matches, input_hash


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=Path(__file__).with_name("m8c_feature_oracle20.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("m8d_track_oracle20.json"),
    )
    parser.add_argument(
        "--temporal-source",
        type=Path,
        default=Path(__file__).parents[2] / "target" / "m8c_feature_raw20_final.json",
        help=(
            "raw upstream M8c dump containing exact temporal ransac_inlier_ids "
            "(default: target/m8c_feature_raw20_final.json)"
        ),
    )
    parser.add_argument("--minimum-track-length", type=int, default=5)
    args = parser.parse_args()
    source = json.loads(args.source.read_text(encoding="utf-8"))
    temporal_source = None
    temporal_source_sha256 = None
    if args.temporal_source.is_file():
        temporal_source_sha256 = sha256_bytes(args.temporal_source.read_bytes())
        expected_raw_hash = source.get("provenance", {}).get("raw_dump_sha256")
        if expected_raw_hash and temporal_source_sha256 != expected_raw_hash:
            raise ValueError(
                "temporal source SHA-256 does not match the M8c provenance "
                f"hash ({temporal_source_sha256} != {expected_raw_hash})"
            )
        temporal_source = json.loads(
            args.temporal_source.read_text(encoding="utf-8")
        )
    summary, _, input_matches, input_hash = build(
        source, args.minimum_track_length, temporal_source
    )
    temporal_source_label = (
        "target/m8c_feature_raw20_final.json"
        if args.temporal_source.name == "m8c_feature_raw20_final.json"
        else args.temporal_source.as_posix()
    )
    artifact = {
        "schema": "basalt-m8d-track-oracle-v1",
        "upstream_commit": UPSTREAM_COMMIT,
        "source_fixture": args.source.name,
        "source_scope": (
            "M8c exact stereo inliers plus exact accepted temporal inliers for "
            "the first 20 TimeCamIds"
        ),
        "source_temporal_dump": temporal_source_label,
        "source_temporal_dump_sha256": temporal_source_sha256,
        "source_match_encoding": (
            "sorted pair keys and sorted FeatureId pairs; each record is "
            "little-endian u64 frame/camera/u64 feature values"
        ),
        "source_match_sha256": input_hash,
        "source_match_pair_count": summary["input_match_pair_count"],
        "source_stereo_pair_count": summary["input_stereo_pair_count"],
        "source_temporal_pair_count": summary["input_temporal_pair_count"],
        "minimum_track_length": args.minimum_track_length,
        "input_matches": input_matches,
        "oracle": summary,
    }
    args.output.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
