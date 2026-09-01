#!/usr/bin/env python3

"""Button-controlled RGB episode recorder for the real robot.

The recorder is intentionally read-only with respect to the robot.  It
subscribes to the XR, home-state, camera and selected robot topics, and calls
the IK node's Trigger services only to request a safe hold/home state change.
It never publishes a robot command.  A new episode is first written below
``output_root/.pending``.  Only a successful episode is moved into the public
episode directory; failed and interrupted episodes are discarded.
"""

import copy
import datetime as dt
import json
from pathlib import Path
import shutil
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image, JointState, Joy
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger
from mit_msgs.msg import MITJointCommands, MITLowState
import yaml


class EpisodeRecorder(Node):
    def __init__(self):
        super().__init__("qiling_episode_recorder")
        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            raise RuntimeError("config_file parameter is required")

        self.config_path = Path(config_file).expanduser().resolve()
        with self.config_path.open("r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream) or {}

        recording = self.config.get("recording", {})
        self.output_root = Path(self.config["output_root"]).expanduser()
        self.storage_id = str(recording.get("storage_id", "mcap"))
        self.jpeg_quality = max(1, min(int(recording.get("jpeg_quality", 90)), 100))
        self.joy_topic = str(recording.get("joy_topic", "/xr/controller_joy"))
        self.start_button = int(recording.get("start_button", 2))
        self.success_button = int(recording.get("success_button", 0))
        self.failure_button = int(recording.get("failure_button", 1))
        self.home_button = int(recording.get("home_button", 3))

        home = self.config.get("home", {})
        self.home_state_topic = str(home.get("state_topic", "/teleop/home_state"))
        self.recording_hold_service = str(
            home.get("hold_service", "/teleop/request_recording_hold")
        )
        self.home_request_service = str(
            home.get("request_service", "/teleop/request_home")
        )
        self.home_ready_code = int(home.get("ready_value", 3))
        self.home_holding_code = int(home.get("holding_value", 5))
        self.home_fault_code = int(home.get("fault_value", 4))

        language = self.config.get("language", {})
        self.language_topic = str(language.get("topic", "/recording/language"))
        self.default_task = str(language.get("default_task", "")).strip()
        self.require_task = bool(language.get("require_nonempty", True))
        self.current_task = self.default_task

        derived = self.config.get("derived_topics", {})
        self.lowstate_topic = str(derived.get("lowstate_topic", "/human_lower_state"))
        self.command_topic = str(derived.get("command_topic", "/human_lower_command"))
        self.observation_joint_topic = str(
            derived.get("observation_joint_state_topic", "/recording/observation/joint_state")
        )
        self.action_joint_topic = str(
            derived.get("action_joint_position_topic", "/recording/action/joint_position")
        )
        self._raw_robot_topics = {self.lowstate_topic, self.command_topic}
        self.joint_names = [str(name) for name in derived.get("joint_names", [])]
        if len(self.joint_names) != 14:
            raise RuntimeError("derived_topics.joint_names must contain 14 arm joint names")

        self.cameras = self.config.get("cameras", {})
        self.topic_specs = [
            (str(spec["name"]), str(spec["type"]))
            for spec in self.config.get("topics", [])
        ]

        self._lock = threading.RLock()
        self._writer = None
        self._event_file = None
        self._episode_dir = None
        self._episode_name = ""
        self._episode_task = ""
        self._active = False
        self._waiting_for_home = False
        self._hold_request_pending = False
        self._hold_confirmed = False
        self._home_request_pending = False
        self._home_state_code = 0
        self._last_buttons = []
        self._bridge = CvBridge()
        self._image_subscriptions = []
        self._topic_subscriptions = []

        self._joy_subscription = self.create_subscription(
            Joy, self.joy_topic, self._joy_callback, qos_profile_sensor_data)
        self._language_subscription = self.create_subscription(
            String, self.language_topic, self._language_callback, qos_profile_sensor_data)
        self._home_state_subscription = self.create_subscription(
            UInt8,
            self.home_state_topic,
            self._home_state_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._hold_client = self.create_client(Trigger, self.recording_hold_service)
        self._home_client = self.create_client(Trigger, self.home_request_service)

        self._lowstate_subscription = self.create_subscription(
            MITLowState, self.lowstate_topic, self._lowstate_callback, qos_profile_sensor_data)
        self._command_subscription = self.create_subscription(
            MITJointCommands, self.command_topic, self._command_callback, qos_profile_sensor_data)

        for camera_name, camera in self.cameras.items():
            image_topic = str(camera["image_topic"])
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    image_topic,
                    lambda message, name=camera_name: self._image_callback(name, message),
                    qos_profile_sensor_data,
                )
            )

        for topic, type_name in self.topic_specs:
            if topic == self.joy_topic:
                self.get_logger().warning(
                    "joy topic is handled by the trigger subscription and will not be duplicated")
                continue
            if topic in self._raw_robot_topics:
                self.get_logger().warning(
                    f"raw robot topic {topic} is used only to derive position data and "
                    "will not be recorded")
                continue
            try:
                message_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as error:
                self.get_logger().error(
                    f"Cannot load configured message type {type_name} for {topic}: {error}")
                continue
            self._topic_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, name=topic: self._topic_callback(name, message),
                    qos_profile_sensor_data,
                )
            )

        self.get_logger().info(
            "Episode recorder ready: left Y starts one episode; left X saves success; "
            "right A discards failure; right B requests direct return to home")
        self.get_logger().info(
            f"Home state={self.home_state_topic} (READY={self.home_ready_code}, "
            f"HOLDING={self.home_holding_code}, FAULT={self.home_fault_code}); "
            f"hold service={self.recording_hold_service}, home service={self.home_request_service}")
        self.get_logger().info(
            f"Language: topic={self.language_topic}, "
            f"default={'<empty>' if not self.default_task else self.default_task!r}, "
            f"required={self.require_task}; task is latched at episode start")
        stream_config = self.config.get("stream", {})
        camera_profiles = ", ".join(
            f"{name}={int(camera.get('width', stream_config.get('width', 640)))}x"
            f"{int(camera.get('height', stream_config.get('height', 480)))}@"
            f"{int(camera.get('fps', stream_config.get('fps', 30)))}Hz"
            for name, camera in self.cameras.items()
        )
        self.get_logger().info(
            f"RGB cameras={len(self.cameras)}, profiles={camera_profiles}, "
            f"resize=disabled, output={self.output_root}")

    @staticmethod
    def _wall_time_string():
        return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")

    @staticmethod
    def _button_pressed(message, index):
        return 0 <= index < len(message.buttons) and message.buttons[index] != 0

    def _write_bag_message_locked(self, topic, message, receive_time_ns):
        if self._writer is None:
            return
        self._writer.write(topic, serialize_message(message), int(receive_time_ns))

    def _write_event_locked(self, event, button=None, reason=None):
        if self._event_file is None:
            return
        record = {
            "event": event,
            "wall_time": self._wall_time_string(),
            "unix_time_ns": time.time_ns(),
            "ros_time_ns": self.get_clock().now().nanoseconds,
        }
        if button is not None:
            record["button"] = button
        if reason is not None:
            record["reason"] = reason
        self._event_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._event_file.flush()

    def _topic_metadata(self):
        topics = [(self.joy_topic, "sensor_msgs/msg/Joy")]
        topics.append((self.language_topic, "std_msgs/msg/String"))
        topics.extend(
            (str(camera["recorded_topic"]), "sensor_msgs/msg/CompressedImage")
            for camera in self.cameras.values()
        )
        topics.extend([
            (self.observation_joint_topic, "sensor_msgs/msg/JointState"),
            (self.action_joint_topic, "sensor_msgs/msg/JointState"),
        ])
        topics.extend(
            (topic, type_name)
            for topic, type_name in self.topic_specs
            if topic not in self._raw_robot_topics
        )

        unique = []
        seen = set()
        for topic, type_name in topics:
            if topic in seen:
                continue
            seen.add(topic)
            unique.append((topic, type_name))
        return unique

    def _write_session_file(self, episode_dir, episode_name, started_ns):
        session = {
            "episode": episode_name,
            "status": "pending",
            "saved": False,
            "started_wall_time": self._wall_time_string(),
            "started_unix_time_ns": started_ns,
            "config_file": str(self.config_path),
            "button_mapping": {
                "left_y_start": self.start_button,
                "left_x_success": self.success_button,
                "right_a_failure": self.failure_button,
                "right_b_request_home": self.home_button,
            },
            "home_control": {
                "state_topic": self.home_state_topic,
                "hold_service": self.recording_hold_service,
                "request_service": self.home_request_service,
                "return_path": "measured_pose_to_home_quintic_without_transition",
            },
            "task": self._episode_task,
            "language_topic": self.language_topic,
            "cameras": copy.deepcopy(self.cameras),
            "stream": copy.deepcopy(self.config.get("stream", {})),
            "topics": [
                (topic, type_name)
                for topic, type_name in self.topic_specs
                if topic not in self._raw_robot_topics
            ],
            "derived_topics": {
                "observation_joint_state_topic": self.observation_joint_topic,
                "action_joint_position_topic": self.action_joint_topic,
                "joint_names": list(self.joint_names),
                "observation_fields": ["position", "velocity"],
                "action_fields": ["position"],
                "torque_recorded": False,
                "low_level_fields_recorded": [],
            },
            "image_storage": {
                "message_type": "sensor_msgs/msg/CompressedImage",
                "format": "jpeg",
                "quality": self.jpeg_quality,
                "timestamp_policy": "header stamp preserved; bag stamp is local receive time",
            },
        }
        with (episode_dir / "session.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(session, stream, allow_unicode=True, sort_keys=False)

    @staticmethod
    def _update_session_result(episode_dir, status, saved):
        session_path = episode_dir / "session.yaml"
        with session_path.open("r", encoding="utf-8") as stream:
            session = yaml.safe_load(stream) or {}
        session["status"] = status
        session["saved"] = bool(saved)
        with session_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(session, stream, allow_unicode=True, sort_keys=False)

    def _home_state_callback(self, message):
        code = int(message.data)
        became_ready = False
        entered_fault = False
        with self._lock:
            previous = self._home_state_code
            self._home_state_code = code
            entered_fault = code == self.home_fault_code and previous != code
            became_ready = (
                code == self.home_ready_code
                and previous != self.home_ready_code
                and self._waiting_for_home
                and not self._active
            )
            if became_ready:
                self._waiting_for_home = False
                self._hold_confirmed = False
                self._home_request_pending = False

        if entered_fault:
            self.get_logger().error(
                "IK home state is FAULT; no new episode can be started")
        elif became_ready:
            self.get_logger().info(
                "Robot reached home; release right B and press left Y to start the next episode")

    def _request_recording_hold(self):
        with self._lock:
            if self._hold_request_pending or self._hold_confirmed:
                return True
            self._hold_request_pending = True

        if not self._hold_client.service_is_ready():
            with self._lock:
                self._hold_request_pending = False
            self.get_logger().error(
                f"Recording hold service is unavailable: {self.recording_hold_service}")
            return False

        try:
            future = self._hold_client.call_async(Trigger.Request())
            future.add_done_callback(self._recording_hold_response)
        except Exception as error:
            with self._lock:
                self._hold_request_pending = False
            self.get_logger().error(f"Failed to call recording hold service: {error}")
            return False
        return True

    def _recording_hold_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            with self._lock:
                self._hold_request_pending = False
            self.get_logger().error(f"Recording hold service failed: {error}")
            return

        with self._lock:
            self._hold_request_pending = False
            if response.success:
                self._hold_confirmed = True
        if response.success:
            self.get_logger().info(
                "Episode boundary is held at the measured pose; press right B to return home")
        else:
            self.get_logger().error(f"Recording hold rejected by IK node: {response.message}")

    def _request_home(self):
        with self._lock:
            if self._home_request_pending:
                return True
            if not self._waiting_for_home:
                return False
            need_hold = not self._hold_confirmed
            if not need_hold:
                if self._home_state_code != self.home_holding_code:
                    self.get_logger().warning(
                        "Home request ignored until IK reports HOLDING_FOR_RECORDING")
                    return False
                self._home_request_pending = True

        if need_hold:
            self.get_logger().warning(
                "Measured-pose hold is not confirmed; retrying hold before right B home request")
            self._request_recording_hold()
            return False

        if not self._home_client.service_is_ready():
            with self._lock:
                self._home_request_pending = False
            self.get_logger().error(
                f"Home request service is unavailable: {self.home_request_service}")
            return False

        try:
            future = self._home_client.call_async(Trigger.Request())
            future.add_done_callback(self._home_response)
        except Exception as error:
            with self._lock:
                self._home_request_pending = False
            self.get_logger().error(f"Failed to call home request service: {error}")
            return False
        self.get_logger().info("Direct measured-pose to home interpolation requested")
        return True

    def _home_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            with self._lock:
                self._home_request_pending = False
            self.get_logger().error(f"Home request service failed: {error}")
            return

        with self._lock:
            self._home_request_pending = False
        if response.success:
            self.get_logger().info(
                "IK accepted direct return home; recorder will wait for READY state")
        else:
            self.get_logger().error(f"Home request rejected by IK node: {response.message}")

    def _start_episode(self, trigger="left_y"):
        with self._lock:
            if self._active:
                self.get_logger().warning("Start requested while an episode is already active")
                return False
            if self._waiting_for_home:
                self.get_logger().warning("Start requested before the robot returned to home")
                return False
            if self._home_state_code != self.home_ready_code:
                self.get_logger().warning(
                    f"Start rejected: IK home state is {self._home_state_code}, "
                    f"expected READY={self.home_ready_code}")
                return False

            task = self.current_task.strip()
            if self.require_task and not task:
                self.get_logger().error(
                    "Recording start rejected: publish a non-empty task on "
                    f"{self.language_topic} or set language.default_task in YAML")
                return False
            if not task:
                task = "unspecified task"

            started_ns = self.get_clock().now().nanoseconds
            stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            episode_name = f"episode_{stamp}"
            self.output_root.mkdir(parents=True, exist_ok=True)
            pending_root = self.output_root / ".pending"
            pending_root.mkdir(parents=True, exist_ok=True)
            episode_dir = pending_root / episode_name
            suffix = 1
            while episode_dir.exists() or (self.output_root / episode_name).exists():
                episode_name = f"episode_{stamp}_{suffix}"
                episode_dir = pending_root / episode_name
                suffix += 1
            episode_dir.mkdir(parents=True, exist_ok=False)
            bag_dir = episode_dir / "rosbag"

            writer = SequentialWriter()
            try:
                writer.open(
                    StorageOptions(uri=str(bag_dir), storage_id=self.storage_id),
                    ConverterOptions(
                        input_serialization_format="cdr",
                        output_serialization_format="cdr",
                    ),
                )
                for topic, type_name in self._topic_metadata():
                    writer.create_topic(
                        TopicMetadata(
                            name=topic,
                            type=type_name,
                            serialization_format="cdr",
                            offered_qos_profiles="",
                        )
                    )
            except Exception as error:
                del writer
                shutil.rmtree(episode_dir, ignore_errors=True)
                self.get_logger().error(f"Cannot open episode MCAP writer: {error}")
                return False

            self._writer = writer
            self._episode_dir = episode_dir
            self._episode_name = episode_name
            self._event_file = (episode_dir / "events.jsonl").open("w", encoding="utf-8")
            self._episode_task = task
            self._active = True
            self._write_session_file(episode_dir, episode_name, started_ns)
            self._write_event_locked("recording_started", trigger)
            language_message = String()
            language_message.data = task
            self._write_bag_message_locked(self.language_topic, language_message, started_ns)
            self.get_logger().info(
                f"Episode started (pending until success): {episode_dir}; task={task!r}")
            return True

    def _finish_episode(self, reason, save, event=None, button=None):
        with self._lock:
            if not self._active:
                return False
            if event is not None:
                self._write_event_locked(event, button)
            self._write_event_locked("recording_stopped", reason=reason)
            self._active = False
            event_file = self._event_file
            writer = self._writer
            episode_dir = self._episode_dir
            episode_name = self._episode_name
            self._event_file = None
            self._writer = None
            self._episode_dir = None
            self._episode_name = ""
            self._episode_task = ""
            if event_file is not None:
                event_file.flush()
                event_file.close()

        # Let rosbag2 finalize metadata before changing/removing the directory.
        del writer
        if episode_dir is None:
            return False

        if save:
            try:
                self._update_session_result(episode_dir, "success", True)
                final_dir = self.output_root / episode_name
                if final_dir.exists():
                    raise RuntimeError(f"episode destination already exists: {final_dir}")
                episode_dir.rename(final_dir)
                self.get_logger().info(
                    f"Episode saved successfully: {final_dir}")
            except Exception as error:
                self.get_logger().error(
                    f"Episode was successful but could not be finalized: {error}; "
                    "pending data has been discarded")
                shutil.rmtree(episode_dir, ignore_errors=True)
                # The episode is closed even when storage finalization fails;
                # the robot must still enter the measured-pose hold path.
                return True
        else:
            shutil.rmtree(episode_dir, ignore_errors=True)
            self.get_logger().info(
                f"Episode discarded ({reason}); no public episode directory was created")
        return True

    def _enter_home_wait(self):
        with self._lock:
            self._waiting_for_home = True
            self._hold_confirmed = False
            self._home_request_pending = False
        self._request_recording_hold()

    @staticmethod
    def _arm_joint_state(stamp, names, positions, velocities=None):
        message = JointState()
        message.header.stamp = stamp
        message.name = list(names)
        message.position = [float(value) for value in positions]
        if velocities is not None:
            message.velocity = [float(value) for value in velocities]
        # Do not populate effort: torque/effort is intentionally excluded from
        # the observation and action recording.
        return message

    def _language_callback(self, message):
        task = str(message.data).strip()
        if not task:
            return
        with self._lock:
            if self._active:
                self.get_logger().warning(
                    "Ignoring language update during an active episode; task is latched")
                return
            self.current_task = task
        self.get_logger().info(f"Next episode language task updated: {task!r}")

    def _lowstate_callback(self, message):
        positions = list(message.joint_states.position)
        velocities = list(message.joint_states.velocity)
        if len(positions) < 26:
            return
        arm_positions = positions[12:26]
        arm_velocities = velocities[12:26] if len(velocities) >= 26 else None
        derived = self._arm_joint_state(
            message.stamp, self.joint_names, arm_positions, arm_velocities)
        with self._lock:
            if self._active:
                self._write_bag_message_locked(
                    self.observation_joint_topic,
                    derived,
                    self.get_clock().now().nanoseconds,
                )

    def _command_callback(self, message):
        commands = list(message.commands)
        if len(commands) < 26:
            return
        arm_positions = [command.pos for command in commands[12:26]]
        derived = self._arm_joint_state(
            message.stamp, self.joint_names, arm_positions)
        with self._lock:
            if self._active:
                self._write_bag_message_locked(
                    self.action_joint_topic,
                    derived,
                    self.get_clock().now().nanoseconds,
                )

    def _joy_callback(self, message):
        with self._lock:
            current = [button != 0 for button in message.buttons]
            previous = self._last_buttons
            self._last_buttons = current

            def rising(index):
                was_pressed = index < len(previous) and previous[index]
                return self._button_pressed(message, index) and not was_pressed

            start_edge = rising(self.start_button)
            home_edge = rising(self.home_button)
            success_edge = rising(self.success_button)
            failure_edge = rising(self.failure_button)
            active_before = self._active
            waiting_before = self._waiting_for_home

        if active_before:
            receive_time_ns = self.get_clock().now().nanoseconds
            with self._lock:
                self._write_bag_message_locked(self.joy_topic, message, receive_time_ns)

            if success_edge:
                if self._finish_episode(
                    "success", save=True, event="episode_success", button="left_x"):
                    self._enter_home_wait()
            elif failure_edge:
                if self._finish_episode(
                    "failure", save=False, event="episode_failure", button="right_a"):
                    self._enter_home_wait()
            elif home_edge:
                self.get_logger().warning(
                    "Right B is ignored during an active episode; mark success/failure first")
            return

        if waiting_before:
            if home_edge:
                self._request_home()
            elif start_edge:
                self.get_logger().warning(
                    "Start ignored: press right B to request home, then wait for READY")
            return

        if start_edge:
            if self._start_episode("left_y"):
                with self._lock:
                    self._write_bag_message_locked(
                        self.joy_topic, message, self.get_clock().now().nanoseconds)

    def _image_callback(self, camera_name, message):
        with self._lock:
            if not self._active:
                return
            camera = self.cameras[camera_name]
            try:
                image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                success, encoded = cv2.imencode(
                    ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                )
                if not success:
                    raise RuntimeError("cv2.imencode returned false")
                compressed = CompressedImage()
                compressed.header = message.header
                compressed.format = "jpeg"
                compressed.data = encoded.tobytes()
                self._write_bag_message_locked(
                    str(camera["recorded_topic"]),
                    compressed,
                    self.get_clock().now().nanoseconds,
                )
            except (CvBridgeError, RuntimeError, ValueError) as error:
                self.get_logger().warning(f"Failed to encode {camera_name} RGB frame: {error}")

    def _topic_callback(self, topic, message):
        with self._lock:
            if self._active:
                self._write_bag_message_locked(topic, message, self.get_clock().now().nanoseconds)

    def stop_for_shutdown(self):
        with self._lock:
            active = self._active
        if active:
            self._finish_episode(
                "node_shutdown", save=False,
                event="recording_interrupted", button="shutdown")
        with self._lock:
            self._waiting_for_home = False
            self._hold_confirmed = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EpisodeRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_for_shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
