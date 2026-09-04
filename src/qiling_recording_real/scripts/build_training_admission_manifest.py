#!/usr/bin/env python3

"""Build a non-destructive training-admission manifest from recorded MCAP episodes.

The manifest never rewrites recordings.  It distinguishes the legacy dual-arm
schema from the current right-arm schema, evaluates the current quality gates,
and emits one full candidate segment for every structurally valid successful
episode. Timing dropout is retained as a warning rather than used to split or
exclude an episode. A later LeRobot converter should consume
``training_segments`` from this manifest instead of globbing every
``episode_*`` directory.
"""

import argparse
import bisect
import json
import math
from pathlib import Path
import re
import sys

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
import yaml


CURRENT_SCHEMA = "right_arm_v1"


def load_yaml(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def load_events(path):
    events = []
    if not path.exists():
        return events
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                events.append(json.loads(line))
    return events


def load_reacquire_ids(quality_guide):
    """Read the guide's explicitly listed serious-image-dropout episodes."""
    path = Path(quality_guide)
    if not path.is_file():
        return set()
    text = path.read_text(encoding="utf-8")
    match = re.search(
        r"以下\s*\d+\s*条出现较明显的图像连续性问题.*?(?=\n四、)",
        text,
        flags=re.DOTALL,
    )
    return set(re.findall(
        r"episode_\d{8}_\d{6}_\d{3}", match.group(0) if match else ""))


def event_status(events):
    names = {event.get("event") for event in events}
    if "episode_success" in names:
        return "success"
    if "episode_failure" in names:
        return "failure"
    return "interrupted"


def read_streams(bag_dir, wanted_topics):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = [topic for topic in wanted_topics if topic not in type_map]
    message_types = {
        topic: get_message(type_map[topic])
        for topic in wanted_topics
        if topic in type_map
    }
    streams = {topic: [] for topic in message_types}
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic in message_types:
            streams[topic].append(
                (int(timestamp_ns), deserialize_message(raw, message_types[topic]))
            )
    for values in streams.values():
        values.sort(key=lambda item: item[0])
    return streams, missing


def stream_metrics(stream):
    if len(stream) < 2:
        return {"count": len(stream), "rate_hz": 0.0, "max_gap_sec": math.inf}
    timestamps = [int(timestamp) for timestamp, _ in stream]
    duration = (timestamps[-1] - timestamps[0]) / 1e9
    return {
        "count": len(stream),
        "rate_hz": (len(stream) - 1) / duration if duration > 0.0 else 0.0,
        "max_gap_sec": max(
            (current - previous) / 1e9
            for previous, current in zip(timestamps, timestamps[1:])
        ),
    }


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def nearest_index(stream, timestamp_ns, tolerance_ns):
    if not stream:
        return None
    timestamps = [item[0] for item in stream]
    index = bisect.bisect_left(timestamps, timestamp_ns)
    candidates = []
    if index < len(stream):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    selected = min(candidates, key=lambda item: abs(timestamps[item] - timestamp_ns))
    if abs(timestamps[selected] - timestamp_ns) > tolerance_ns:
        return None
    return selected


def validate_camera_stream(camera, stream, validate_jpeg):
    expected_width = int(camera.get("width", 640))
    expected_height = int(camera.get("height", 480))
    invalid = 0
    if validate_jpeg:
        for _, message in stream:
            if str(message.format).lower() != "jpeg":
                invalid += 1
                continue
            image = cv2.imdecode(
                np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (expected_height, expected_width):
                invalid += 1
    return invalid


def validate_state_action(joint_names, observation, action, gripper):
    atomic = (
        bool(observation)
        and len(observation) == len(action) == len(gripper)
        and [timestamp for timestamp, _ in observation]
        == [timestamp for timestamp, _ in action]
        == [timestamp for timestamp, _ in gripper]
    )
    invalid_observation = sum(
        len(message.name) != 7
        or list(message.name) != joint_names
        or len(message.position) != 7
        or len(message.velocity) != 7
        or not finite(message.position)
        or not finite(message.velocity)
        for _, message in observation
    )
    invalid_action = sum(
        len(message.name) != 7
        or list(message.name) != joint_names
        or len(message.position) != 7
        or bool(message.velocity)
        or bool(message.effort)
        or not finite(message.position)
        for _, message in action
    )
    invalid_gripper = sum(
        int(message.data) not in (0, 1) for _, message in gripper
    )
    return {
        "atomic": atomic,
        "invalid_observation": invalid_observation,
        "invalid_action": invalid_action,
        "invalid_gripper": invalid_gripper,
    }


def camera_stats_clean(session):
    for stats in session.get("camera_stats", {}).values():
        if sum(int(stats.get(key, 0)) for key in (
                "encode_errors", "write_errors", "dropped_writer_queue")):
            return False
    stats = session.get("state_action_stats", {})
    return (
        int(stats.get("samples_dropped_writer_queue", 0)) == 0
        and int(stats.get("write_errors", 0)) == 0
    )


def contiguous_segments(valid_samples, minimum_segment_sec):
    segments = []
    start = None
    for index, sample in enumerate(valid_samples + [None]):
        is_valid = sample is not None and sample["valid"]
        if is_valid and start is None:
            start = index
        if not is_valid and start is not None:
            end = index
            length = end - start
            first = valid_samples[start]
            last = valid_samples[end - 1]
            duration_sec = (last["time_ns"] - first["time_ns"]) / 1e9
            if length >= 2 and duration_sec >= minimum_segment_sec:
                segments.append({
                    "start_time_ns": first["time_ns"],
                    "end_time_ns": last["time_ns"],
                    "frames": length,
                    "duration_sec": duration_sec,
                })
            start = None
    return segments


def synchronized_segments(streams, topics, camera_tolerance_ns,
                          state_tolerance_ns, maximum_head_gap_ns,
                          minimum_segment_sec):
    required = [topics[name] for name in (
        "head", "left", "right", "observation", "action", "gripper")]
    if any(not streams.get(topic) for topic in required):
        return [], {"candidate_frames": 0, "valid_frames": 0, "invalid_frames": 0}

    samples = []
    previous_time_ns = None
    # The head camera is the policy/video master.  Do not force a synthetic
    # fixed-rate grid here: otherwise normal camera phase offsets and timestamp
    # jitter create artificial gaps.  A real excessive head-frame gap still
    # terminates the segment below.
    for current_ns, _ in streams[topics["head"]]:
        missing = []
        if (previous_time_ns is not None
                and current_ns - previous_time_ns > maximum_head_gap_ns):
            missing.append("head_gap")
        for name in ("left", "right"):
            if nearest_index(streams[topics[name]], current_ns, camera_tolerance_ns) is None:
                missing.append(name)
        for name in ("observation", "action", "gripper"):
            if nearest_index(streams[topics[name]], current_ns, state_tolerance_ns) is None:
                missing.append(name)
        samples.append({
            "time_ns": current_ns,
            "valid": not missing,
            "missing": missing,
        })
        previous_time_ns = current_ns

    return contiguous_segments(samples, minimum_segment_sec), {
        "candidate_frames": len(samples),
        "valid_frames": sum(sample["valid"] for sample in samples),
        "invalid_frames": sum(not sample["valid"] for sample in samples),
        "invalid_reasons": {
            name: sum(name in sample["missing"] for sample in samples)
            for name in ("head_gap", "left", "right", "observation", "action", "gripper")
        },
    }


def evaluate_episode(episode_dir, thresholds, expected_cameras, args, reacquire_ids):
    session = load_yaml(episode_dir / "session.yaml")
    events = load_events(episode_dir / "events.jsonl")
    result = {
        "source_episode": episode_dir.name,
        "source_path": str(episode_dir),
        "task": str(session.get("task", "")).strip(),
        "session_status": session.get("status"),
        "event_status": event_status(events),
        "saved": bool(session.get("saved")),
        "segments": [],
        "reasons": [],
        "warnings": [],
    }
    cameras = session.get("cameras", {})
    derived = session.get("derived_topics", {})
    joint_names = [str(name) for name in derived.get("joint_names", [])]
    schema_current = (
        derived.get("arm_scope") == "right_only"
        and len(joint_names) == 7
        and "right_gripper_action_topic" in derived
        and set(cameras) == {"head", "left", "right"}
    )
    if not schema_current:
        result["tier"] = "excluded_legacy_schema"
        result["reasons"].append("not_current_right_arm_v1_schema")
        return result

    topics = {
        "head": str(cameras["head"]["recorded_topic"]),
        "left": str(cameras["left"]["recorded_topic"]),
        "right": str(cameras["right"]["recorded_topic"]),
        "observation": str(derived["observation_joint_state_topic"]),
        "action": str(derived["action_joint_position_topic"]),
        "gripper": str(derived["right_gripper_action_topic"]),
    }
    streams, missing_topics = read_streams(episode_dir / "rosbag", list(topics.values()))
    result["topics"] = topics
    result["stream_metrics"] = {
        name: stream_metrics(streams.get(topic, [])) for name, topic in topics.items()
    }
    if missing_topics:
        result["reasons"].append("missing_topics:" + ",".join(missing_topics))
    for name, topic in topics.items():
        if not streams.get(topic):
            result["reasons"].append(f"empty_topic:{name}")
    if session.get("status") != "success" or not session.get("saved"):
        result["reasons"].append("session_not_saved_success")
    if result["event_status"] != "success":
        result["reasons"].append("events_not_success")
    if not result["task"]:
        result["reasons"].append("empty_task")

    camera_invalid = {}
    for name in ("head", "left", "right"):
        camera = cameras[name]
        expected = expected_cameras[name]
        profile_ok = (
            int(camera.get("source_width", 0)) == int(expected.get("source_width", expected["width"]))
            and int(camera.get("source_height", 0)) == int(expected.get("source_height", expected["height"]))
            and float(camera.get("source_fps", 0.0)) == float(expected.get("source_fps", expected["fps"]))
            and int(camera.get("width", 0)) == int(expected["width"])
            and int(camera.get("height", 0)) == int(expected["height"])
            and float(camera.get("fps", 0.0)) == float(expected["fps"])
        )
        if not profile_ok:
            result["reasons"].append(f"camera_profile:{name}")
        camera_invalid[name] = validate_camera_stream(
            camera, streams.get(topics[name], []), not args.skip_jpeg_validation)
        if camera_invalid[name]:
            result["reasons"].append(f"camera_jpeg_or_shape:{name}")
    result["camera_invalid_frames"] = camera_invalid

    validation = validate_state_action(
        joint_names,
        streams.get(topics["observation"], []),
        streams.get(topics["action"], []),
        streams.get(topics["gripper"], []),
    )
    result["state_action_validation"] = validation
    if not validation["atomic"]:
        result["warnings"].append("state_action_not_atomic")
    if any(validation[key] for key in (
            "invalid_observation", "invalid_action", "invalid_gripper")):
        result["reasons"].append("state_action_schema")
    if not camera_stats_clean(session):
        result["warnings"].append("recorder_faults")

    state_metrics = result["stream_metrics"]["observation"]
    state_strict = (
        state_metrics["rate_hz"] >= 50.0 * thresholds["state_action_min_rate_ratio"]
        and state_metrics["max_gap_sec"] <= thresholds["state_action_max_gap_sec"]
    )
    if not state_strict:
        result["warnings"].append("state_action_rate_or_gap")
    camera_strict = True
    for name in ("head", "left", "right"):
        metrics = result["stream_metrics"][name]
        if not (
            metrics["rate_hz"] >= float(expected_cameras[name]["fps"])
            * thresholds["image_min_rate_ratio"]
            and metrics["max_gap_sec"] <= thresholds["image_max_gap_sec"]
        ):
            camera_strict = False

    if not camera_strict:
        result["warnings"].append("camera_rate_or_gap")

    if result["reasons"]:
        result["tier"] = "rejected"
        return result

    # Keep each successful, structurally valid source episode intact.  The
    # export stage aligns missing streams by nearest timestamp instead of
    # dropping intervals because temporal dropout is advisory in this mode.
    head_stream = streams[topics["head"]]
    start_time_ns = head_stream[0][0]
    end_time_ns = head_stream[-1][0]
    result["segments"] = [{
        "start_time_ns": start_time_ns,
        "end_time_ns": end_time_ns,
        "frames": len(head_stream),
        "duration_sec": max(0.0, (end_time_ns - start_time_ns) / 1e9),
    }]
    result["synchronization"] = {
        "policy": "all_recorded_frames_nearest_timestamp_alignment",
        "candidate_frames": len(head_stream),
        "valid_frames": len(head_stream),
        "invalid_frames": 0,
    }
    result["tier"] = "all_recorded"
    return result


def write_markdown(path, manifest):
    summary = manifest["summary"]
    lines = [
        "# Training admission manifest",
        "",
        f"- Generated at: {manifest['generated_at_utc']}",
        f"- Input root: `{manifest['input_root']}`",
        f"- Schema accepted: `{CURRENT_SCHEMA}`",
        "",
        "## Summary",
        "",
        "| Tier | Episodes | Training segments |",
        "|---|---:|---:|",
    ]
    for tier in ("all_recorded", "rejected", "excluded_legacy_schema"):
        lines.append(
            f"| {tier} | {summary['episodes_by_tier'].get(tier, 0)} | "
            f"{summary['segments_by_tier'].get(tier, 0)} |"
        )
    lines.extend([
        "",
        "All structurally valid successful episodes are listed in "
        "`training_segments`. Timing/queue dropout is reported as a warning and "
        "does not block conversion in this collection mode.",
        "",
        "## Training segments",
        "",
        "| Segment | Tier | Source episode | Frames | Duration (s) |",
        "|---|---|---|---:|---:|",
    ])
    for segment in manifest["training_segments"]:
        lines.append(
            f"| {segment['segment_id']} | {segment['tier']} | "
            f"{segment['source_episode']} | {segment['frames']} | "
            f"{segment['duration_sec']:.2f} |"
        )
    with path.open("w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


def parse_args():
    try:
        default_config = (
            Path(get_package_share_directory("qiling_recording_real"))
            / "config" / "real_recording.yaml"
        )
    except Exception:
        default_config = Path(__file__).resolve().parents[1] / "config" / "real_recording.yaml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_root", help="Directory containing episode_* folders")
    parser.add_argument(
        "--output", default=None,
        help="Output JSON path (default: INPUT_ROOT/training_admission_manifest.json)")
    parser.add_argument("--config-file", default=str(default_config))
    parser.add_argument(
        "--quality-guide", default=None,
        help="Optional RECORDING_QUALITY_GUIDE.txt; its serious-dropout list is excluded.")
    parser.add_argument(
        "--fps", type=float, default=None,
        help="Output policy rate (default: head camera fps in config_file)")
    parser.add_argument("--camera-sync-tolerance-sec", type=float, default=0.025)
    parser.add_argument("--state-action-sync-tolerance-sec", type=float, default=0.015)
    parser.add_argument("--minimum-segment-sec", type=float, default=3.0)
    parser.add_argument(
        "--skip-jpeg-validation", action="store_true",
        help="Skip full JPEG decode/shape validation; use only for a quick dry run.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.fps is not None and args.fps <= 0.0:
        raise RuntimeError("fps must be positive")
    if args.minimum_segment_sec <= 0.0:
        raise RuntimeError("minimum-segment-sec must be positive")
    input_root = Path(args.input_root).expanduser().resolve()
    if not input_root.is_dir():
        raise RuntimeError(f"Input root does not exist: {input_root}")
    output = Path(args.output).expanduser().resolve() if args.output else (
        input_root / "training_admission_manifest.json")
    quality_guide = Path(args.quality_guide).expanduser().resolve() if args.quality_guide else (
        input_root.parent / "RECORDING_QUALITY_GUIDE.txt")
    reacquire_ids = load_reacquire_ids(quality_guide)
    config = load_yaml(args.config_file)
    expected_cameras = config.get("cameras", {})
    if set(expected_cameras) != {"head", "left", "right"}:
        raise RuntimeError("config_file must define head, left, and right cameras")
    if args.fps is None:
        args.fps = float(expected_cameras["head"]["fps"])
    quality = config.get("quality_check", {})
    thresholds = {
        "image_min_rate_ratio": float(quality.get("image_min_rate_ratio", 0.95)),
        "image_max_gap_sec": float(quality.get("image_max_gap_sec", 0.10)),
        "state_action_min_rate_ratio": float(quality.get("state_action_min_rate_ratio", 0.96)),
        "state_action_max_gap_sec": float(quality.get("state_action_max_gap_sec", 0.05)),
    }

    episodes = []
    training_segments = []
    review_segments = []
    reacquire_segments = []
    for episode_dir in sorted(input_root.glob("episode_*")):
        if not ((episode_dir / "session.yaml").is_file() and (episode_dir / "rosbag").is_dir()):
            continue
        print(f"Inspecting {episode_dir.name}", flush=True)
        result = evaluate_episode(
            episode_dir, thresholds, expected_cameras, args, reacquire_ids)
        for index, segment in enumerate(result["segments"]):
            segment_id = f"{result['source_episode']}__seg_{index:03d}"
            segment.update({
                "segment_id": segment_id,
                "source_episode": result["source_episode"],
                "source_path": result["source_path"],
                "tier": result["tier"],
                "task": result["task"],
                "schema": CURRENT_SCHEMA,
                "fps": args.fps,
            })
            if result["tier"] == "all_recorded":
                training_segments.append(segment)
        episodes.append(result)

    now = __import__("datetime").datetime.now(
        __import__("datetime").timezone.utc).isoformat()
    summary = {"episodes_by_tier": {}, "segments_by_tier": {}}
    for result in episodes:
        tier = result["tier"]
        summary["episodes_by_tier"][tier] = summary["episodes_by_tier"].get(tier, 0) + 1
        summary["segments_by_tier"][tier] = summary["segments_by_tier"].get(tier, 0) + len(result["segments"])
    for name, segments in (("training", training_segments), ("review", review_segments),
                           ("reacquire", reacquire_segments)):
        summary[f"{name}_segments"] = len(segments)
        summary[f"{name}_frames"] = sum(segment["frames"] for segment in segments)
        summary[f"{name}_duration_sec"] = sum(segment["duration_sec"] for segment in segments)
    manifest = {
        "manifest_version": 1,
        "generated_at_utc": now,
        "input_root": str(input_root),
        "quality_guide": str(quality_guide) if quality_guide.is_file() else None,
        "schema": CURRENT_SCHEMA,
        "policy_fps": args.fps,
        "jpeg_validation": {
            "performed_by_this_run": not args.skip_jpeg_validation,
            "note": (
                "JPEG decoding and dimensions were validated by this run."
                if not args.skip_jpeg_validation else
                "Skipped by this run; rely only on a separately recorded quality-check result."
            ),
        },
        "synchronization": {
            "camera_tolerance_sec": args.camera_sync_tolerance_sec,
            "state_action_tolerance_sec": args.state_action_sync_tolerance_sec,
            "minimum_segment_sec": args.minimum_segment_sec,
            "timebase": "MCAP record timestamps (the recorder's selected local receive/sample time)",
        },
        "quality_thresholds": thresholds,
        "summary": summary,
        "episodes": episodes,
        "training_segments": training_segments,
        "review_segments": review_segments,
        "reacquire_segments": reacquire_segments,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    markdown = output.with_suffix(".md")
    write_markdown(markdown, manifest)
    print(
        f"Wrote {output} and {markdown}: "
        f"{summary['training_segments']} admitted segments, "
        f"{summary['review_segments']} review segments, "
        f"{summary['reacquire_segments']} reacquire-only segments.")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
