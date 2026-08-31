#!/usr/bin/env python3

"""Convert qiling_recording_real MCAP episodes to LeRobot Dataset v3.

The converter intentionally runs offline.  It creates one regular LeRobot
dataset through LeRobotDataset.create/add_frame/save_episode/finalize, which
keeps the output schema owned by the installed LeRobot version instead of
hand-writing parquet or metadata files.
"""

import argparse
import bisect
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
import yaml


def load_config(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def read_events(path):
    events = []
    if not path.exists():
        return events
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                events.append(json.loads(line))
    return events


def episode_status(events):
    names = {event.get("event") for event in events}
    if "episode_success" in names:
        return "success"
    if "episode_failure" in names:
        return "failure"
    return "interrupted"


def read_bag(bag_dir, topic_names, storage_id="mcap"):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_map[topic])
        for topic in topic_names
        if topic in type_map
    }
    streams = {topic: [] for topic in message_types}
    while reader.has_next():
        topic, raw, record_time_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(raw, message_types[topic])
        streams[topic].append((int(record_time_ns), message))
    for values in streams.values():
        values.sort(key=lambda item: item[0])
    return streams


def nearest(stream, time_ns, tolerance_ns):
    if not stream:
        return None
    times = [item[0] for item in stream]
    index = bisect.bisect_left(times, time_ns)
    candidates = []
    if index < len(stream):
        candidates.append(stream[index])
    if index > 0:
        candidates.append(stream[index - 1])
    selected = min(candidates, key=lambda item: abs(item[0] - time_ns))
    return selected[1] if abs(selected[0] - time_ns) <= tolerance_ns else None


def decode_rgb(message):
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return None
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def pose_vector(message):
    return np.asarray([
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ], dtype=np.float32)


def joint_vector(message, use_velocity=False):
    values = message.velocity if use_velocity else message.position
    return np.asarray(values, dtype=np.float32)


def vector_feature(size, names):
    return {
        "dtype": "float32",
        "shape": (size,),
        "names": list(names),
    }


def image_feature(use_videos, height, width):
    return {
        "dtype": "video" if use_videos else "image",
        "shape": (3, height, width),
        "names": ["channels", "height", "width"],
    }


def action_names(mode, joint_names):
    eef_names = [
        "left_x", "left_y", "left_z", "left_qx", "left_qy", "left_qz", "left_qw",
        "right_x", "right_y", "right_z", "right_qx", "right_qy", "right_qz", "right_qw",
    ]
    if mode == "joint":
        return list(joint_names)
    if mode == "eef":
        return eef_names
    return [f"eef_{name}" for name in eef_names] + [f"joint_{name}" for name in joint_names]


def build_features(config, joint_names, action_mode, include_alternates):
    stream = config.get("stream", {})
    width = int(stream.get("width", 640))
    height = int(stream.get("height", 480))
    use_videos = bool(config.get("use_videos", True))
    features = {
        "observation.images.head": image_feature(use_videos, height, width),
        "observation.images.left": image_feature(use_videos, height, width),
        "observation.images.right": image_feature(use_videos, height, width),
        "observation.state": vector_feature(14, joint_names),
        "observation.velocity": vector_feature(14, [f"d_{name}" for name in joint_names]),
        "observation.eef_pose": vector_feature(
            14,
            action_names("eef", joint_names),
        ),
        "action": vector_feature(
            len(action_names(action_mode, joint_names)),
            action_names(action_mode, joint_names),
        ),
    }
    if include_alternates:
        features["action.joint_position"] = vector_feature(14, joint_names)
        features["action.eef_pose"] = vector_feature(
            14,
            action_names("eef", joint_names),
        )
    return features


def convert_episode(dataset, episode_dir, config, action_mode, include_alternates):
    session_path = episode_dir / "session.yaml"
    with session_path.open("r", encoding="utf-8") as stream:
        session = yaml.safe_load(stream) or {}
    task = str(session.get("task", "")).strip()
    if not task:
        raise RuntimeError(f"Episode {episode_dir.name} has no non-empty task")

    cameras = session["cameras"]
    derived = session["derived_topics"]
    left_state = "/teleop/left_wrist_state"
    right_state = "/teleop/right_wrist_state"
    left_target = "/teleop/left_wrist_target"
    right_target = "/teleop/right_wrist_target"
    topic_names = [
        str(camera["recorded_topic"]) for camera in cameras.values()
    ] + [
        left_state,
        right_state,
        left_target,
        right_target,
        str(derived["observation_joint_state_topic"]),
        str(derived["action_joint_position_topic"]),
    ]
    streams = read_bag(
        episode_dir / "rosbag",
        topic_names,
        storage_id=str(config.get("storage_id", "mcap")),
    )

    head_topic = str(cameras["head"]["recorded_topic"])
    left_image_topic = str(cameras["left"]["recorded_topic"])
    right_image_topic = str(cameras["right"]["recorded_topic"])
    head_stream = streams.get(head_topic, [])
    if not head_stream:
        raise RuntimeError(f"Episode {episode_dir.name} has no head RGB frames")

    tolerance_ns = int(float(config.get("sync_tolerance_sec", 0.025)) * 1e9)
    joint_names = [str(name) for name in derived["joint_names"]]
    observation_topic = str(derived["observation_joint_state_topic"])
    action_joint_topic = str(derived["action_joint_position_topic"])
    frames = []
    skipped = 0

    for master_time_ns, _ in head_stream:
        head = nearest(head_stream, master_time_ns, tolerance_ns)
        left_image = nearest(streams.get(left_image_topic, []), master_time_ns, tolerance_ns)
        right_image = nearest(streams.get(right_image_topic, []), master_time_ns, tolerance_ns)
        observation_joint = nearest(streams.get(observation_topic, []), master_time_ns, tolerance_ns)
        action_joint = nearest(streams.get(action_joint_topic, []), master_time_ns, tolerance_ns)
        left_pose = nearest(streams.get(left_state, []), master_time_ns, tolerance_ns)
        right_pose = nearest(streams.get(right_state, []), master_time_ns, tolerance_ns)
        left_target_pose = nearest(streams.get(left_target, []), master_time_ns, tolerance_ns)
        right_target_pose = nearest(streams.get(right_target, []), master_time_ns, tolerance_ns)

        required = [
            head, left_image, right_image, observation_joint, action_joint,
            left_pose, right_pose, left_target_pose, right_target_pose,
        ]
        if any(value is None for value in required):
            skipped += 1
            continue

        images = [decode_rgb(head), decode_rgb(left_image), decode_rgb(right_image)]
        if any(image is None for image in images):
            skipped += 1
            continue

        observed_positions = joint_vector(observation_joint)
        observed_velocities = joint_vector(observation_joint, use_velocity=True)
        commanded_joints = joint_vector(action_joint)
        if len(observed_positions) != 14 or len(observed_velocities) != 14 or len(commanded_joints) != 14:
            skipped += 1
            continue
        observed_eef = np.concatenate([pose_vector(left_pose), pose_vector(right_pose)])
        commanded_eef = np.concatenate([
            pose_vector(left_target_pose), pose_vector(right_target_pose)
        ])
        if action_mode == "joint":
            canonical_action = commanded_joints
        elif action_mode == "eef":
            canonical_action = commanded_eef
        else:
            canonical_action = np.concatenate([commanded_eef, commanded_joints])

        frame = {
            "observation.images.head": images[0],
            "observation.images.left": images[1],
            "observation.images.right": images[2],
            "observation.state": observed_positions,
            "observation.velocity": observed_velocities,
            "observation.eef_pose": observed_eef,
            "action": canonical_action,
            "task": task,
        }
        if include_alternates:
            frame["action.joint_position"] = commanded_joints
            frame["action.eef_pose"] = commanded_eef
        frames.append(frame)

    if not frames:
        raise RuntimeError(f"Episode {episode_dir.name} has no complete synchronized frames")

    for frame in frames:
        dataset.add_frame(frame)
    dataset.save_episode(parallel_encoding=False)
    return {
        "episode": episode_dir.name,
        "status": episode_status(read_events(episode_dir / "events.jsonl")),
        "frames": len(frames),
        "skipped_frames": skipped,
        "task": task,
    }


def parse_args():
    default_config = (
        Path(get_package_share_directory("qiling_recording_real"))
        / "config"
        / "lerobot_conversion.yaml"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-file", default=str(default_config))
    return parser.parse_args()


def main():
    args = parse_args()
    conversion = load_config(args.config_file)
    input_root = Path(conversion["input_root"]).expanduser()
    output_root = Path(conversion["output_root"]).expanduser()
    if output_root.exists():
        raise RuntimeError(
            f"Output root already exists: {output_root}; choose a new output_root "
            "to avoid silently mixing datasets"
        )

    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as error:
        raise RuntimeError(
            "LeRobot is not installed. Install a version with Dataset v3 support "
            "(official docs require lerobot >= 0.4.0), then rerun conversion."
        ) from error
    if not hasattr(LeRobotDataset, "create"):
        raise RuntimeError("Installed LeRobot does not expose LeRobotDataset.create()")

    episodes = []
    for episode_dir in sorted(input_root.glob("episode_*")):
        if (episode_dir / "session.yaml").exists() and (episode_dir / "rosbag").exists():
            episodes.append(episode_dir)
    if not episodes:
        raise RuntimeError(f"No complete episodes found under {input_root}")

    first_session = load_config(episodes[0] / "session.yaml")
    joint_names = [str(name) for name in first_session["derived_topics"]["joint_names"]]
    action_mode = str(conversion.get("action_mode", "joint"))
    if action_mode not in ("joint", "eef", "both"):
        raise RuntimeError("action_mode must be joint, eef or both")
    include_failed = bool(conversion.get("include_failed", True))
    include_alternates = bool(conversion.get("include_alternate_actions", True))
    features = build_features(conversion, joint_names, action_mode, include_alternates)

    dataset = LeRobotDataset.create(
        repo_id=str(conversion.get("repo_id", "local/qiling_television")),
        fps=int(conversion.get("fps", 20)),
        features=features,
        root=output_root,
        robot_type="qiling_dual_arm",
        use_videos=bool(conversion.get("use_videos", True)),
        tolerance_s=float(conversion.get("sync_tolerance_sec", 0.025)),
        image_writer_processes=0,
        image_writer_threads=4,
    )

    converted = []
    try:
        for episode_dir in episodes:
            status = episode_status(read_events(episode_dir / "events.jsonl"))
            if status == "failure" and not include_failed:
                continue
            converted.append(
                convert_episode(
                    dataset,
                    episode_dir,
                    conversion,
                    action_mode,
                    include_alternates,
                )
            )
        if not converted:
            raise RuntimeError("No episodes selected for conversion")
    finally:
        # finalize() is required by LeRobot to close parquet/video writers and
        # write valid metadata footers, including tasks and statistics.
        dataset.finalize()

    with (output_root / "conversion_manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "action_mode": action_mode,
                "include_alternate_actions": include_alternates,
                "observation_fields": [
                    "rgb",
                    "joint_position",
                    "joint_velocity",
                    "eef_pose",
                ],
                "low_level_fields": [],
                "episodes": converted,
            },
            stream,
            ensure_ascii=False,
            indent=2,
        )
    print(f"Converted {len(converted)} episodes to {output_root}")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
