//! M8d track-builder parity against the checked-in first-20 M8c exact matches.
//!
//! The source fixture contains ten stereo pairs, i.e. twenty `TimeCamId`s, and
//! M8d embeds the exact accepted temporal inlier vectors from the same pinned
//! upstream diagnostic dump.  All vectors are consumed directly as upstream
//! `MatchData.inliers`; no descriptor or image reprocessing is hidden in this
//! gate.

use std::collections::BTreeMap;

use serde_json::Value;
use visloc_basalt::mapper::{build_track_oracle, MatchData, Matches, TimeCamId, Track};

const M8C: &str = include_str!("../../../benchmarks/basalt/m8c_feature_oracle20.json");
const M8D: &str = include_str!("../../../benchmarks/basalt/m8d_track_oracle20.json");

fn image_id(value: &Value) -> TimeCamId {
    TimeCamId::new(
        value["frame_id"].as_u64().expect("frame_id"),
        value["cam_id"].as_u64().expect("cam_id") as u16,
    )
}

fn inliers(value: &Value) -> Vec<(u64, u64)> {
    value
        .as_array()
        .expect("inlier array")
        .iter()
        .map(|entry| {
            if let Some(text) = entry.as_str() {
                let mut fields = text.split_whitespace();
                (
                    fields.next().unwrap().parse().unwrap(),
                    fields.next().unwrap().parse().unwrap(),
                )
            } else {
                (
                    entry[0].as_u64().expect("left feature"),
                    entry[1].as_u64().expect("right feature"),
                )
            }
        })
        .collect()
}

fn canonical_tracks(value: &Value) -> Vec<Track> {
    value
        .as_array()
        .expect("canonical observations")
        .iter()
        .map(|track| Track {
            id: track["id"].as_u64().expect("track id"),
            observations: track["observations"]
                .as_array()
                .expect("track observations")
                .iter()
                .map(|observation| {
                    (
                        image_id(&observation["image"]),
                        observation["feature_id"].as_u64().expect("feature id"),
                    )
                })
                .collect(),
        })
        .collect()
}

#[test]
fn m8d_first20_inliers_match_portable_oracle_summary() {
    let source: Value = serde_json::from_str(M8C).expect("M8c fixture JSON");
    let expected: Value = serde_json::from_str(M8D).expect("M8d fixture JSON");

    // M8c intentionally keeps the 180 temporal inlier vectors as hashes, but
    // records the exact raw-dump hash.  M8d embeds those exact upstream input
    // pairs so this gate remains portable and never fabricates temporal edges.
    assert_eq!(
        expected["source_fixture"].as_str(),
        Some("m8c_feature_oracle20.json")
    );
    assert_eq!(
        expected["source_temporal_dump_sha256"].as_str(),
        source["provenance"]["raw_dump_sha256"].as_str()
    );
    assert_eq!(
        expected["source_temporal_dump_sha256"].as_str(),
        Some("7fc7c6f3a74dd14c6ca786c061a692a98f2ac50dada26253181f811d3228a485")
    );

    let mut matches = Matches::new();
    for pair in expected["input_matches"]
        .as_array()
        .expect("M8d input matches")
    {
        matches.insert(
            (image_id(&pair["left"]), image_id(&pair["right"])),
            MatchData::from_inliers(inliers(&pair["inliers"])),
        );
    }
    assert_eq!(matches.len(), 190);

    // The ten stereo records must be the exact M8c inlier vectors, not a
    // separately regenerated/self-generated edge set.
    for pair in source["stereo"]["pairs"].as_array().expect("stereo pairs") {
        let key = (image_id(&pair["left"]), image_id(&pair["right"]));
        assert_eq!(
            matches[&key].inliers,
            inliers(&pair["essential_inlier_ids"])
        );
    }

    let summary = build_track_oracle(&matches, 5);
    let expected = &expected["oracle"];
    assert_eq!(
        summary.node_count,
        expected["node_count"].as_u64().unwrap() as usize
    );
    assert_eq!(
        summary.component_count_before,
        expected["component_count_before"].as_u64().unwrap() as usize
    );
    assert_eq!(
        summary.component_count_after,
        expected["component_count_after"].as_u64().unwrap() as usize
    );
    assert_eq!(
        summary.rejected_conflict_ids,
        expected["rejected_conflict_ids"]
            .as_array()
            .unwrap()
            .iter()
            .map(|value| value.as_u64().unwrap())
            .collect::<Vec<_>>()
    );
    assert_eq!(
        summary.rejected_short_ids,
        expected["rejected_short_ids"]
            .as_array()
            .unwrap()
            .iter()
            .map(|value| value.as_u64().unwrap())
            .collect::<Vec<_>>()
    );
    assert_eq!(
        summary.rejected_track_ids,
        expected["rejected_track_ids"]
            .as_array()
            .unwrap()
            .iter()
            .map(|value| value.as_u64().unwrap())
            .collect::<Vec<_>>()
    );
    assert_eq!(
        summary.exported_track_count,
        expected["exported_track_count"].as_u64().unwrap() as usize
    );
    let expected_histogram = expected["track_length_histogram"]
        .as_object()
        .unwrap()
        .iter()
        .map(|(length, count)| {
            (
                length.parse::<usize>().unwrap(),
                count.as_u64().unwrap() as usize,
            )
        })
        .collect::<BTreeMap<_, _>>();
    assert_eq!(summary.track_length_histogram, expected_histogram);
    assert_eq!(
        summary.canonical_hash,
        expected["canonical_hash"].as_u64().unwrap()
    );
    assert_eq!(
        summary.canonical_observations,
        canonical_tracks(&expected["canonical_observations"])
    );
}
