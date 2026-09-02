#!/usr/bin/env python3

"""Offline quality check for one qiling_recording_real episode.

The checker opens an existing MCAP only.  It never initializes ROS nodes,
subscribes to topics, publishes commands, or changes episode contents.
"""

import argparse
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
import yaml


class Report:
    def __init__(self):
        self.failures = 0

    def check(self, label, condition, detail):
        status = "PASS" if condition else "FAIL"
        print(f"[{status}] {label}: {detail}")
        if not condition:
            self.failures += 1


def load_yaml(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def rate_and_max_gap(messages):
    """Return rate and largest adjacent time gap for a list of bag records."""
    if len(messages) < 2:
        return 0.0, math.inf
    timestamps = sorted(int(item[0]) for item in messages)
    duration_sec = (timestamps[-1] - timestamps[0]) / 1e9
    rate_hz = (len(timestamps) - 1) / duration_sec if duration_sec > 0.0 else 0.0
    max_gap_sec = max(
        (current - previous) / 1e9
        for previous, current in zip(timestamps, timestamps[1:])
    )
    return rate_hz, max_gap_sec


def finite_values(values):
    return all(math.isfinite(float(value)) for value in values)


def read_streams(bag_dir, wanted_topics):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    missing_topics = [topic for topic in wanted_topics if topic not in type_map]
    message_types = {
        topic: get_message(type_map[topic])
        for topic in wanted_topics
        if topic in type_map
    }
    streams = {topic: [] for topic in message_types}
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic in message_types:
            streams[topic].append((int(timestamp_ns), deserialize_message(
                raw, message_types[topic])))
    return streams, missing_topics


def check_camera(report, name, camera, stream, thresholds, session_stats):
    width = int(camera["width"])
    height = int(camera["height"])
    target_fps = float(camera["fps"])
    source_width = int(camera.get("source_width", width))
    source_height = int(camera.get("source_height", height))
    source_fps = float(camera.get("source_fps", target_fps))
    report.check(
        f"camera/{name}/profile",
        (width, height, target_fps) == (640, 480, 30.0)
        and (source_width, source_height, source_fps) == (640, 480, 30.0),
        f"source={source_width}x{source_height}@{source_fps:g}Hz, "
        f"recorded={width}x{height}@{target_fps:g}Hz",
    )
    count = len(stream)
    rate_hz, max_gap_sec = rate_and_max_gap(stream)
    report.check(
        f"camera/{name}/frames", count > 0,
        f"count={count}")
    report.check(
        f"camera/{name}/rate",
        rate_hz >= target_fps * thresholds["image_min_rate_ratio"],
        f"observed={rate_hz:.2f}Hz, target={target_fps:.2f}Hz, "
        f"minimum={target_fps * thresholds['image_min_rate_ratio']:.2f}Hz",
    )
    report.check(
        f"camera/{name}/max_gap",
        max_gap_sec <= thresholds["image_max_gap_sec"],
        f"max_gap={max_gap_sec:.4f}s, allowed={thresholds['image_max_gap_sec']:.4f}s",
    )

    invalid = 0
    for _, message in stream:
        if str(message.format).lower() != "jpeg":
            invalid += 1
            continue
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            invalid += 1
    report.check(
        f"camera/{name}/jpeg_and_shape", invalid == 0,
        f"invalid={invalid}/{count}, expected=JPEG {width}x{height}",
    )

    stats = session_stats.get(name, {})
    faults = sum(int(stats.get(field, 0)) for field in (
        "encode_errors", "write_errors", "dropped_writer_queue"))
    report.check(
        f"camera/{name}/recorder_faults", faults == 0,
        f"encode_errors={stats.get('encode_errors', 0)}, "
        f"write_errors={stats.get('write_errors', 0)}, "
        f"dropped_writer_queue={stats.get('dropped_writer_queue', 0)}",
    )


def check_joint_messages(report, joint_names, observation, action, gripper):
    invalid_observation = sum(
        len(message.name) != 7
        or list(message.name) != joint_names
        or len(message.position) != 7
        or len(message.velocity) != 7
        or not finite_values(message.position)
        or not finite_values(message.velocity)
        for _, message in observation
    )
    invalid_action = sum(
        len(message.name) != 7
        or list(message.name) != joint_names
        or len(message.position) != 7
        or bool(message.velocity)
        or bool(message.effort)
        or not finite_values(message.position)
        for _, message in action
    )
    invalid_gripper = sum(
        int(message.data) not in (0, 1) for _, message in gripper)
    report.check(
        "state_action/schema",
        invalid_observation == 0 and invalid_action == 0 and invalid_gripper == 0,
        f"invalid_observation={invalid_observation}/{len(observation)}, "
        f"invalid_action={invalid_action}/{len(action)}, "
        f"invalid_gripper={invalid_gripper}/{len(gripper)}",
    )


def check_state_action(report, session, streams, thresholds):
    derived = session["derived_topics"]
    observation_topic = str(derived["observation_joint_state_topic"])
    action_topic = str(derived["action_joint_position_topic"])
    gripper_topic = str(derived["right_gripper_action_topic"])
    observation = streams.get(observation_topic, [])
    action = streams.get(action_topic, [])
    gripper = streams.get(gripper_topic, [])
    target_hz = float(session.get("recording_runtime", {}).get(
        "state_action_rate_hz", 50.0))
    count_match = (
        len(observation) > 0
        and len(observation) == len(action) == len(gripper)
        and [timestamp for timestamp, _ in observation]
        == [timestamp for timestamp, _ in action]
        == [timestamp for timestamp, _ in gripper]
    )
    report.check(
        "state_action/atomic_batches", count_match,
        f"observation={len(observation)}, action={len(action)}, gripper={len(gripper)}",
    )
    rate_hz, max_gap_sec = rate_and_max_gap(observation)
    report.check(
        "state_action/rate",
        rate_hz >= target_hz * thresholds["state_action_min_rate_ratio"],
        f"observed={rate_hz:.2f}Hz, target={target_hz:.2f}Hz, "
        f"minimum={target_hz * thresholds['state_action_min_rate_ratio']:.2f}Hz",
    )
    report.check(
        "state_action/max_gap",
        max_gap_sec <= thresholds["state_action_max_gap_sec"],
        f"max_gap={max_gap_sec:.4f}s, allowed={thresholds['state_action_max_gap_sec']:.4f}s",
    )
    check_joint_messages(
        report,
        [str(name) for name in derived["joint_names"]],
        observation,
        action,
        gripper,
    )

    stats = session.get("state_action_stats", {})
    faults = int(stats.get("samples_dropped_writer_queue", 0)) + int(
        stats.get("write_errors", 0))
    report.check(
        "state_action/recorder_faults", faults == 0,
        f"dropped_writer_queue={stats.get('samples_dropped_writer_queue', 0)}, "
        f"write_errors={stats.get('write_errors', 0)}, "
        f"missed_deadlines={stats.get('scheduler_missed_deadlines', 0)}",
    )


def parse_args():
    try:
        default_config = (
            Path(get_package_share_directory("qiling_recording_real"))
            / "config" / "real_recording.yaml"
        )
    except Exception:
        # Direct execution from the source tree, before ament has been built.
        default_config = (
            Path(__file__).resolve().parents[1]
            / "config" / "real_recording.yaml"
        )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir", help="Saved episode directory containing session.yaml")
    parser.add_argument(
        "--config-file",
        default=str(default_config),
        help="Recording YAML that supplies quality_check thresholds",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    episode_dir = Path(args.episode_dir).expanduser().resolve()
    session_path = episode_dir / "session.yaml"
    bag_dir = episode_dir / "rosbag"
    if not session_path.is_file() or not bag_dir.is_dir():
        raise RuntimeError(
            f"Expected {episode_dir}/session.yaml and {episode_dir}/rosbag")

    session = load_yaml(session_path)
    config = load_yaml(args.config_file)
    thresholds = {
        "image_min_rate_ratio": float(
            config.get("quality_check", {}).get("image_min_rate_ratio", 0.95)),
        "image_max_gap_sec": float(
            config.get("quality_check", {}).get("image_max_gap_sec", 0.10)),
        "state_action_min_rate_ratio": float(
            config.get("quality_check", {}).get("state_action_min_rate_ratio", 0.96)),
        "state_action_max_gap_sec": float(
            config.get("quality_check", {}).get("state_action_max_gap_sec", 0.05)),
    }
    cameras = session.get("cameras", {})
    derived = session.get("derived_topics", {})
    if set(cameras) != {"head", "left", "right"}:
        raise RuntimeError("session.yaml must define head, left, and right cameras")
    required_derived = (
        "observation_joint_state_topic",
        "action_joint_position_topic",
        "right_gripper_action_topic",
        "joint_names",
    )
    if any(key not in derived for key in required_derived):
        raise RuntimeError("session.yaml has incomplete right-arm derived_topics")

    camera_topics = {
        name: str(camera["recorded_topic"]) for name, camera in cameras.items()
    }
    wanted_topics = list(camera_topics.values()) + [
        str(session["language_topic"]),
        str(derived["observation_joint_state_topic"]),
        str(derived["action_joint_position_topic"]),
        str(derived["right_gripper_action_topic"]),
    ]
    streams, missing_topics = read_streams(bag_dir, wanted_topics)
    report = Report()

    print(f"Episode: {episode_dir}")
    print("Expected: RGB=640x480@30Hz JPEG, right-arm observation/action/gripper=50Hz")
    report.check(
        "episode/status",
        session.get("status") == "success" and bool(session.get("saved")),
        f"status={session.get('status')!r}, saved={session.get('saved')!r}",
    )
    report.check(
        "task/language",
        bool(str(session.get("task", "")).strip())
        and len(streams.get(str(session["language_topic"]), [])) == 1,
        f"task={session.get('task')!r}, language_messages="
        f"{len(streams.get(str(session['language_topic']), []))}",
    )
    report.check(
        "bag/topics",
        not missing_topics,
        "all required topics present" if not missing_topics
        else "missing=" + ", ".join(missing_topics),
    )

    camera_stats = session.get("camera_stats", {})
    for name, camera in cameras.items():
        check_camera(
            report, name, camera, streams.get(camera_topics[name], []),
            thresholds, camera_stats)
    check_state_action(report, session, streams, thresholds)

    writer_stats = session.get("writer_stats", {})
    report.check(
        "writer/auxiliary_queue",
        int(writer_stats.get("auxiliary_messages_dropped_writer_queue", 0)) == 0,
        "dropped=" + str(
            writer_stats.get("auxiliary_messages_dropped_writer_queue", 0)),
    )
    if report.failures:
        print(f"OVERALL: FAIL ({report.failures} required checks failed)")
        return 2
    print("OVERALL: PASS (episode satisfies the current RGB and 50Hz right-arm requirements)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
