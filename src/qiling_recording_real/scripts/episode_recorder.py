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
from dataclasses import dataclass
import datetime as dt
import json
from pathlib import Path
from queue import Empty, Full, Queue
import shutil
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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


@dataclass(frozen=True)
class WriteBatch:
    """ROS messages to be serialized and written by the only MCAP writer thread."""

    generation: int
    timestamp_ns: int
    messages: tuple
    kind: str
    camera_name: str = ""


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
        self.state_action_rate_hz = max(
            1.0, float(recording.get("state_action_rate_hz", 50.0)))
        self.state_action_period_sec = 1.0 / self.state_action_rate_hz
        self.state_action_period_ns = int(1e9 / self.state_action_rate_hz)
        self.state_action_timeout_sec = max(
            0.02, float(recording.get("state_action_timeout_sec", 0.10)))
        self.state_action_timeout_ns = int(self.state_action_timeout_sec * 1e9)
        self.state_action_write_queue_depth = max(
            1, int(recording.get("state_action_write_queue_depth", 2048)))
        self.auxiliary_write_queue_depth = max(
            1, int(recording.get("auxiliary_write_queue_depth", 2048)))
        self.image_write_queue_depth = max(
            1, int(recording.get("image_write_queue_depth", 1024)))
        self.writer_drain_warning_sec = max(
            1.0, float(recording.get("writer_drain_warning_sec", 5.0)))
        self.camera_timeout_sec = max(
            0.1, float(recording.get("camera_timeout_sec", 0.5)))
        self.camera_timeout_ns = int(self.camera_timeout_sec * 1e9)
        self.image_queue_depth = max(
            1, int(recording.get("image_queue_depth", 2)))
        self.executor_threads = max(
            4, int(recording.get("executor_threads", 4)))
        self.camera_diagnostics_period_sec = max(
            0.5, float(recording.get("camera_diagnostics_period_sec", 2.0)))

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
        self.o6_trigger_state_topic = str(
            derived.get("o6_trigger_state_topic", "/teleop/o6_trigger_state"))
        self.observation_joint_topic = str(
            derived.get(
                "observation_joint_state_topic",
                "/recording/observation/right_joint_state",
            )
        )
        self.action_joint_topic = str(
            derived.get(
                "action_joint_position_topic",
                "/recording/action/right_joint_position",
            )
        )
        self.right_gripper_action_topic = str(
            derived.get(
                "right_gripper_action_topic",
                "/recording/action/right_gripper_closed",
            )
        )
        self._raw_robot_topics = {self.lowstate_topic, self.command_topic}
        self.joint_names = [str(name) for name in derived.get("joint_names", [])]
        if len(self.joint_names) != 7:
            raise RuntimeError(
                "derived_topics.joint_names must contain 7 right-arm joint names")

        self.cameras = self.config.get("cameras", {})
        if not self.cameras:
            raise RuntimeError("at least one RGB camera must be configured")
        self.image_resize_mode = str(
            recording.get("image_resize_mode", "none")
        ).strip().lower()
        if self.image_resize_mode != "none":
            raise RuntimeError(
                "recording.image_resize_mode must be none; camera profiles must match output")
        self._camera_output_profiles = {}
        self._camera_output_period_ns = {}
        self._camera_rate_limit_enabled = {}
        for camera_name, camera in self.cameras.items():
            output_width = int(camera.get("width", 640))
            output_height = int(camera.get("height", 480))
            output_fps = int(camera.get("fps", 20))
            source_width = int(camera.get("source_width", output_width))
            source_height = int(camera.get("source_height", output_height))
            source_fps = int(camera.get("source_fps", output_fps))
            if output_width <= 0 or output_height <= 0 or output_fps <= 0:
                raise RuntimeError(
                    f"Invalid output profile for camera {camera_name}: "
                    f"{output_width}x{output_height}@{output_fps}")
            if (source_width, source_height, source_fps) != (
                    output_width, output_height, output_fps):
                raise RuntimeError(
                    f"Camera {camera_name} must use one native recording profile; "
                    f"source={source_width}x{source_height}@{source_fps}, "
                    f"output={output_width}x{output_height}@{output_fps}")
            self._camera_output_profiles[camera_name] = {
                "width": output_width,
                "height": output_height,
                "fps": output_fps,
            }
            self._camera_output_period_ns[camera_name] = int(1e9 / output_fps)
            self._camera_rate_limit_enabled[camera_name] = False
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
        self._episode_generation = 0
        self._episode_started_ns = 0
        self._episode_started_steady_ns = 0
        self._active = False
        self._writer_accepting_generation = 0
        self._waiting_for_home = False
        self._hold_request_pending = False
        self._hold_confirmed = False
        self._home_request_pending = False
        self._home_state_code = 0
        self._last_buttons = []
        self._latest_observation = None
        self._latest_observation_received_ns = 0
        self._latest_arm_action = None
        self._latest_arm_action_received_ns = 0
        self._latest_right_gripper_closed = 0
        self._latest_gripper_received_ns = 0
        self._state_action_samples_written = 0
        self._state_action_samples_enqueued = 0
        self._state_action_samples_skipped = 0
        self._state_action_samples_dropped_writer_queue = 0
        self._state_action_scheduler_missed_deadlines = 0
        self._state_action_write_errors = 0
        self._auxiliary_messages_dropped_writer_queue = 0
        self._image_subscriptions = []
        self._topic_subscriptions = []
        self._camera_callback_groups = {}
        self._camera_diagnostic_callback_group = MutuallyExclusiveCallbackGroup()
        self._camera_last_seen_ns = {
            camera_name: 0 for camera_name in self.cameras
        }
        self._camera_last_input_shape = {
            camera_name: (0, 0) for camera_name in self.cameras
        }
        self._camera_last_output_slot = {
            camera_name: -1 for camera_name in self.cameras
        }
        self._camera_stats = self._empty_camera_stats()
        self._image_queues = {
            camera_name: Queue(maxsize=self.image_queue_depth)
            for camera_name in self.cameras
        }
        self._camera_bridges = {
            camera_name: CvBridge() for camera_name in self.cameras
        }
        self._camera_worker_stop = threading.Event()
        self._camera_workers = []
        self._state_action_write_queue = Queue(
            maxsize=self.state_action_write_queue_depth)
        self._auxiliary_write_queue = Queue(
            maxsize=self.auxiliary_write_queue_depth)
        self._image_write_queue = Queue(
            maxsize=self.image_write_queue_depth)
        self._writer_stop = threading.Event()
        self._writer_wakeup = threading.Event()
        self._writer_thread = None
        self._state_sampler_stop = threading.Event()
        self._state_sampler_wakeup = threading.Event()
        self._state_sampler_thread = None

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
        self._o6_trigger_subscription = self.create_subscription(
            UInt8,
            self.o6_trigger_state_topic,
            self._o6_trigger_callback,
            qos_profile_sensor_data,
        )

        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for camera_name, camera in self.cameras.items():
            image_topic = str(camera["image_topic"])
            callback_group = MutuallyExclusiveCallbackGroup()
            self._camera_callback_groups[camera_name] = callback_group
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    image_topic,
                    lambda message, name=camera_name: self._image_callback(name, message),
                    camera_qos,
                    callback_group=callback_group,
                )
            )

        for topic, type_name in self.topic_specs:
            if topic == self.joy_topic:
                self.get_logger().warning(
                    "joy topic is handled by the trigger subscription and will not be duplicated")
                continue
            if topic == self.o6_trigger_state_topic:
                self.get_logger().info(
                    "O6 trigger topic is handled by the right-gripper action subscription")
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

        self._camera_diagnostics_timer = self.create_timer(
            self.camera_diagnostics_period_sec,
            self._camera_diagnostics_callback,
            callback_group=self._camera_diagnostic_callback_group,
        )
        self._start_camera_workers()
        self._start_writer_worker()
        self._start_state_sampler()

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
        camera_profiles = ", ".join(
            f"{name}="
            f"{int(camera.get('source_width', camera.get('width', 640)))}x"
            f"{int(camera.get('source_height', camera.get('height', 480)))}@"
            f"{int(camera.get('source_fps', camera.get('fps', 30)))}Hz->"
            f"{self._camera_output_profiles[name]['width']}x"
            f"{self._camera_output_profiles[name]['height']}@"
            f"{self._camera_output_profiles[name]['fps']}Hz"
            for name, camera in self.cameras.items()
        )
        self.get_logger().info(
            f"RGB cameras={len(self.cameras)}, profiles={camera_profiles}, "
            f"resize={self.image_resize_mode}, rate_limit="
            f"{'enabled when source fps exceeds output fps' if any(self._camera_rate_limit_enabled.values()) else 'disabled'}, "
            f"QoS=RELIABLE/KEEP_LAST(1), "
            f"queue_depth={self.image_queue_depth}, workers={len(self._camera_workers)}, "
            f"executor_threads={self.executor_threads}, output={self.output_root}")
        self.get_logger().info(
            f"Right-arm observation/action sampled by a dedicated steady-clock "
            f"thread at {self.state_action_rate_hz:.1f} Hz; "
            f"source freshness timeout={self.state_action_timeout_sec:.2f}s; "
            "right gripper action=0(open)/1(closed)")
        self.get_logger().info(
            f"Camera freshness timeout={self.camera_timeout_sec:.2f}s; "
            "camera frame-rate validation on success is disabled")

    @staticmethod
    def _wall_time_string():
        return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")

    def _empty_camera_stats(self):
        return {
            camera_name: {
                "received": 0,
                "enqueued": 0,
                "encoded": 0,
                "writer_enqueued": 0,
                "written": 0,
                "dropped_rate_limit": 0,
                "dropped_queue": 0,
                "dropped_writer_queue": 0,
                "dropped_stale": 0,
                "encode_errors": 0,
                "write_errors": 0,
                "input_width": 0,
                "input_height": 0,
                "output_width": self._camera_output_profiles[camera_name]["width"],
                "output_height": self._camera_output_profiles[camera_name]["height"],
                "target_fps": self._camera_output_profiles[camera_name]["fps"],
            }
            for camera_name in self.cameras
        }

    def _prepare_camera_image(self, camera_name, image):
        profile = self._camera_output_profiles[camera_name]
        target_width = profile["width"]
        target_height = profile["height"]
        if image is None or image.ndim < 2:
            raise ValueError("decoded camera image is empty")

        source_height, source_width = image.shape[:2]
        if source_width != target_width or source_height != target_height:
            raise RuntimeError(
                f"camera output is {source_width}x{source_height}, expected native "
                f"{target_width}x{target_height}; refusing to crop or resize")
        return image, source_width, source_height

    def _start_camera_workers(self):
        for camera_name in self.cameras:
            worker = threading.Thread(
                target=self._camera_worker,
                args=(camera_name,),
                name=f"qiling_rgb_{camera_name}_worker",
                daemon=True,
            )
            worker.start()
            self._camera_workers.append(worker)

    def _stop_camera_workers(self):
        self._camera_worker_stop.set()
        for worker in self._camera_workers:
            worker.join(timeout=2.0)
            if worker.is_alive():
                self.get_logger().warning(
                    f"Camera worker did not stop within timeout: {worker.name}")

    def _start_writer_worker(self):
        self._writer_thread = threading.Thread(
            target=self._writer_worker,
            name="qiling_mcap_writer",
            daemon=True,
        )
        self._writer_thread.start()

    def _stop_writer_worker(self):
        self._writer_stop.set()
        self._writer_wakeup.set()
        if self._writer_thread is not None:
            self._writer_thread.join(timeout=3.0)
            if self._writer_thread.is_alive():
                self.get_logger().warning("MCAP writer thread did not stop within timeout")

    def _start_state_sampler(self):
        self._state_sampler_thread = threading.Thread(
            target=self._state_sampler_worker,
            name="qiling_state_action_sampler",
            daemon=True,
        )
        self._state_sampler_thread.start()

    def _stop_state_sampler(self):
        self._state_sampler_stop.set()
        self._state_sampler_wakeup.set()
        if self._state_sampler_thread is not None:
            self._state_sampler_thread.join(timeout=3.0)
            if self._state_sampler_thread.is_alive():
                self.get_logger().warning(
                    "State/action sampler thread did not stop within timeout")

    def _writer_is_open_locked(self, generation):
        return (
            self._writer is not None
            and self._writer_accepting_generation == generation
            and self._episode_generation == generation
        )

    def _enqueue_write_batch_locked(self, queue, batch):
        if not self._writer_is_open_locked(batch.generation):
            return False
        try:
            queue.put_nowait(batch)
        except Full:
            if batch.kind == "state_action":
                self._state_action_samples_dropped_writer_queue += 1
            elif batch.kind == "image":
                self._camera_stats[batch.camera_name]["dropped_writer_queue"] += 1
            else:
                self._auxiliary_messages_dropped_writer_queue += 1
            return False
        self._writer_wakeup.set()
        return True

    def _enqueue_message_locked(self, topic, message, timestamp_ns):
        generation = self._episode_generation
        batch = WriteBatch(
            generation=generation,
            timestamp_ns=int(timestamp_ns),
            messages=((str(topic), message),),
            kind="auxiliary",
        )
        return self._enqueue_write_batch_locked(self._auxiliary_write_queue, batch)

    def _pop_next_write_batch(self):
        for queue in (
            self._state_action_write_queue,
            self._auxiliary_write_queue,
            self._image_write_queue,
        ):
            try:
                return queue, queue.get_nowait()
            except Empty:
                continue
        return None, None

    def _writer_worker(self):
        while True:
            queue, batch = self._pop_next_write_batch()
            if batch is None:
                if self._writer_stop.is_set():
                    return
                self._writer_wakeup.wait(timeout=0.02)
                self._writer_wakeup.clear()
                continue
            try:
                self._write_batch(batch)
            finally:
                queue.task_done()

    def _write_batch(self, batch):
        with self._lock:
            if not self._writer_is_open_locked(batch.generation):
                return
            writer = self._writer

        try:
            for topic, message in batch.messages:
                writer.write(topic, serialize_message(message), int(batch.timestamp_ns))
        except Exception as error:
            with self._lock:
                if batch.kind == "state_action":
                    self._state_action_write_errors += 1
                    error_count = self._state_action_write_errors
                elif batch.kind == "image":
                    stats = self._camera_stats[batch.camera_name]
                    stats["write_errors"] += 1
                    error_count = stats["write_errors"]
                else:
                    self._auxiliary_messages_dropped_writer_queue += 1
                    error_count = self._auxiliary_messages_dropped_writer_queue
            if error_count == 1 or error_count % 100 == 0:
                self.get_logger().error(
                    f"MCAP writer failed for {batch.kind} batch "
                    f"(count={error_count}): {error}")
            return

        with self._lock:
            if batch.kind == "state_action":
                self._state_action_samples_written += 1
            elif batch.kind == "image":
                self._camera_stats[batch.camera_name]["written"] += 1

    def _wait_for_queue_drain(self, queue, label):
        next_warning = time.monotonic() + self.writer_drain_warning_sec
        while True:
            with queue.all_tasks_done:
                if queue.unfinished_tasks == 0:
                    return
                queue.all_tasks_done.wait(timeout=0.1)
            if time.monotonic() >= next_warning:
                self.get_logger().warning(
                    f"Waiting for {label} queue to drain "
                    f"(unfinished={queue.unfinished_tasks})")
                next_warning = time.monotonic() + self.writer_drain_warning_sec

    def _drain_recording_queues(self):
        for camera_name, queue in self._image_queues.items():
            self._wait_for_queue_drain(queue, f"{camera_name} JPEG input")
        self._writer_wakeup.set()
        self._wait_for_queue_drain(
            self._state_action_write_queue, "state/action writer")
        self._wait_for_queue_drain(
            self._auxiliary_write_queue, "auxiliary writer")
        self._wait_for_queue_drain(self._image_write_queue, "image writer")

    def _state_sampler_worker(self):
        while not self._state_sampler_stop.is_set():
            self._state_sampler_wakeup.wait(timeout=0.1)
            if self._state_sampler_stop.is_set():
                return

            with self._lock:
                if not self._active:
                    # Clear while holding the same lock used by _start_episode.
                    # A new episode therefore cannot lose its wake-up event here.
                    self._state_sampler_wakeup.clear()
                    continue
                generation = self._episode_generation
                start_steady_ns = self._episode_started_steady_ns
                start_ros_ns = self._episode_started_ns

            next_deadline_ns = start_steady_ns
            while not self._state_sampler_stop.is_set():
                now_steady_ns = time.monotonic_ns()
                if now_steady_ns < next_deadline_ns:
                    self._state_sampler_stop.wait(
                        (next_deadline_ns - now_steady_ns) / 1e9)
                    continue

                if now_steady_ns - next_deadline_ns >= self.state_action_period_ns:
                    missed = (now_steady_ns - next_deadline_ns) // self.state_action_period_ns
                    with self._lock:
                        if self._active and self._episode_generation == generation:
                            self._state_action_scheduler_missed_deadlines += int(missed)
                    next_deadline_ns += int(missed) * self.state_action_period_ns

                sample_ros_ns = start_ros_ns + (next_deadline_ns - start_steady_ns)
                self._sample_state_action_at(
                    generation, sample_ros_ns, time.monotonic_ns())
                next_deadline_ns += self.state_action_period_ns

                with self._lock:
                    if not self._active or self._episode_generation != generation:
                        break
            self._state_sampler_wakeup.clear()

    def _camera_worker(self, camera_name):
        image_queue = self._image_queues[camera_name]
        bridge = self._camera_bridges[camera_name]

        while not self._camera_worker_stop.is_set():
            try:
                episode_generation, message, receive_time_ns = image_queue.get(timeout=0.1)
            except Empty:
                continue

            try:
                with self._lock:
                    current_episode = self._writer_is_open_locked(episode_generation)
                    if (
                        not current_episode
                        and episode_generation == self._episode_generation
                    ):
                        self._camera_stats[camera_name]["dropped_stale"] += 1
                if not current_episode:
                    continue

                try:
                    image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                    image, input_width, input_height = self._prepare_camera_image(
                        camera_name, image)
                    success, encoded = cv2.imencode(
                        ".jpg",
                        image,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                    )
                    if not success:
                        raise RuntimeError("cv2.imencode returned false")
                except (CvBridgeError, RuntimeError, ValueError, cv2.error) as error:
                    with self._lock:
                        if episode_generation == self._episode_generation:
                            stats = self._camera_stats[camera_name]
                            stats["encode_errors"] += 1
                            error_count = stats["encode_errors"]
                        else:
                            error_count = 0
                    if error_count == 1 or (error_count > 0 and error_count % 100 == 0):
                        self.get_logger().warning(
                            f"Failed to encode {camera_name} RGB frame "
                            f"(count={error_count}): {error}")
                    continue

                compressed = CompressedImage()
                compressed.header = message.header
                compressed.format = "jpeg"
                compressed.data = encoded.tobytes()

                with self._lock:
                    stats = self._camera_stats[camera_name]
                    current_episode = self._writer_is_open_locked(episode_generation)
                    if not current_episode:
                        if episode_generation == self._episode_generation:
                            stats["dropped_stale"] += 1
                        continue
                    stats["input_width"] = input_width
                    stats["input_height"] = input_height
                    stats["encoded"] += 1
                    batch = WriteBatch(
                        generation=episode_generation,
                        timestamp_ns=receive_time_ns,
                        messages=((
                            str(self.cameras[camera_name]["recorded_topic"]),
                            compressed,
                        ),),
                        kind="image",
                        camera_name=camera_name,
                    )
                    if self._enqueue_write_batch_locked(
                        self._image_write_queue, batch):
                        stats["writer_enqueued"] += 1
            finally:
                image_queue.task_done()

    def _camera_diagnostics_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            if not self._active:
                return
            elapsed_sec = max((now_ns - self._episode_started_ns) / 1e9, 1e-6)
            stats = copy.deepcopy(self._camera_stats)

        reports = []
        for camera_name, camera_stats in stats.items():
            reports.append(
                f"{camera_name}: recv={camera_stats['received']} "
                f"enc={camera_stats['encoded']} write={camera_stats['written']} "
                f"drop_rate={camera_stats['dropped_rate_limit']} "
                f"drop_q={camera_stats['dropped_queue']} "
                f"drop_writer={camera_stats['dropped_writer_queue']} "
                f"drop_stale={camera_stats['dropped_stale']} "
                f"err={camera_stats['encode_errors'] + camera_stats['write_errors']} "
                f"write_hz={camera_stats['written'] / elapsed_sec:.1f} "
                f"q={self._image_queues[camera_name].qsize()}"
            )
        self.get_logger().info("RGB episode stats | " + " | ".join(reports))

    @staticmethod
    def _button_pressed(message, index):
        return 0 <= index < len(message.buttons) and message.buttons[index] != 0

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
            (self.right_gripper_action_topic, "std_msgs/msg/UInt8"),
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
                "right_gripper_action_topic": self.right_gripper_action_topic,
                "joint_names": list(self.joint_names),
                "observation_fields": ["position", "velocity"],
                "action_fields": ["position"],
                "right_gripper_action": {
                    "type": "std_msgs/msg/UInt8",
                    "source_topic": self.o6_trigger_state_topic,
                    "source_mask": "0x02",
                    "open_value": 0,
                    "closed_value": 1,
                    "feedback_available": False,
                },
                "arm_scope": "right_only",
                "torque_recorded": False,
                "low_level_fields_recorded": [],
            },
            "image_storage": {
                "message_type": "sensor_msgs/msg/CompressedImage",
                "format": "jpeg",
                "quality": self.jpeg_quality,
                "output_resize_mode": self.image_resize_mode,
                "output_rate_policy": "native camera profile; no crop, resize, rotation, or software rate limiting",
                "timestamp_policy": "source header stamp preserved; bag stamp is selected frame local receive time",
                "input_qos": "RELIABLE/KEEP_LAST(1)",
                "queue_depth_per_camera": self.image_queue_depth,
                "worker_count": len(self._camera_workers),
                "writer_queue_depth": self.image_write_queue_depth,
                "frame_rate_validation_on_success": False,
            },
            "recording_runtime": {
                "executor_threads": self.executor_threads,
                "state_action_rate_hz": self.state_action_rate_hz,
                "state_action_timeout_sec": self.state_action_timeout_sec,
                "state_action_sampler": "dedicated_steady_clock_thread",
                "state_action_writer_queue_depth": self.state_action_write_queue_depth,
                "auxiliary_writer_queue_depth": self.auxiliary_write_queue_depth,
                "image_writer_queue_depth": self.image_write_queue_depth,
                "writer": "single_mcap_writer_thread_with_state_action_priority",
                "camera_timeout_sec": self.camera_timeout_sec,
                "camera_diagnostics_period_sec": self.camera_diagnostics_period_sec,
            },
        }
        with (episode_dir / "session.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(session, stream, allow_unicode=True, sort_keys=False)

    @staticmethod
    def _update_session_result(
        episode_dir,
        status,
        saved,
        duration_sec=None,
        camera_stats=None,
        state_action_stats=None,
        writer_stats=None,
    ):
        session_path = episode_dir / "session.yaml"
        with session_path.open("r", encoding="utf-8") as stream:
            session = yaml.safe_load(stream) or {}
        session["status"] = status
        session["saved"] = bool(saved)
        if duration_sec is not None:
            session["duration_sec"] = float(duration_sec)
        if camera_stats is not None:
            session["camera_stats"] = copy.deepcopy(camera_stats)
        if state_action_stats is not None:
            session["state_action_stats"] = copy.deepcopy(state_action_stats)
        if writer_stats is not None:
            session["writer_stats"] = copy.deepcopy(writer_stats)
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

            now_steady_ns = time.monotonic_ns()
            stale_cameras = []
            for camera_name, last_seen_ns in self._camera_last_seen_ns.items():
                if last_seen_ns <= 0:
                    stale_cameras.append(f"{camera_name}=never")
                    continue
                age_ns = now_steady_ns - last_seen_ns
                if age_ns > self.camera_timeout_ns:
                    age_sec = age_ns / 1e9
                    stale_cameras.append(f"{camera_name}={age_sec:.3f}s")
            if stale_cameras:
                self.get_logger().error(
                    "Recording start rejected because RGB input is stale: "
                    + ", ".join(stale_cameras)
                )
                return False

            profile_mismatches = []
            for camera_name, (input_width, input_height) in self._camera_last_input_shape.items():
                expected = self._camera_output_profiles[camera_name]
                if (input_width, input_height) != (
                        expected["width"], expected["height"]):
                    profile_mismatches.append(
                        f"{camera_name}={input_width}x{input_height}, "
                        f"expected {expected['width']}x{expected['height']}")
            if profile_mismatches:
                self.get_logger().error(
                    "Recording start rejected because RGB profiles do not match the "
                    "configured native recording profile: "
                    + "; ".join(profile_mismatches)
                )
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
            started_steady_ns = time.monotonic_ns()
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
            self._episode_generation += 1
            generation = self._episode_generation
            self._episode_started_ns = started_ns
            self._episode_started_steady_ns = started_steady_ns
            self._state_action_samples_written = 0
            self._state_action_samples_enqueued = 0
            self._state_action_samples_skipped = 0
            self._state_action_samples_dropped_writer_queue = 0
            self._state_action_scheduler_missed_deadlines = 0
            self._state_action_write_errors = 0
            self._auxiliary_messages_dropped_writer_queue = 0
            self._camera_stats = self._empty_camera_stats()
            self._camera_last_output_slot = {
                camera_name: -1 for camera_name in self.cameras
            }
            self._active = True
            self._writer_accepting_generation = generation
            self._write_session_file(episode_dir, episode_name, started_ns)
            self._write_event_locked("recording_started", trigger)
            language_message = String()
            language_message.data = task
            self._enqueue_message_locked(self.language_topic, language_message, started_ns)
            self._state_sampler_wakeup.set()
            self.get_logger().info(
                f"Episode started (pending until success): {episode_dir}; task={task!r}")
            return True

    def _finish_episode(self, reason, save, event=None, button=None):
        with self._lock:
            if not self._active:
                return False
            finished_steady_ns = time.monotonic_ns()
            duration_sec = max(
                (finished_steady_ns - self._episode_started_steady_ns) / 1e9, 0.0)
            if event is not None:
                self._write_event_locked(event, button)
            self._write_event_locked("recording_stopped", reason=reason)
            self._active = False
            event_file = self._event_file
            episode_dir = self._episode_dir
            episode_name = self._episode_name
            self._event_file = None
            if event_file is not None:
                event_file.flush()
                event_file.close()

        if episode_dir is None:
            return False

        # Stop accepting new callbacks, then let all already accepted JPEG and
        # MCAP batches finish before closing the writer.  This is what keeps a
        # success button from truncating the final camera/state samples.
        self._drain_recording_queues()

        with self._lock:
            camera_stats = copy.deepcopy(self._camera_stats)
            state_action_stats = {
                "target_rate_hz": self.state_action_rate_hz,
                "scheduler": "dedicated_steady_clock_thread",
                "samples_enqueued": self._state_action_samples_enqueued,
                "samples_written": self._state_action_samples_written,
                "samples_skipped_stale_or_missing": self._state_action_samples_skipped,
                "samples_dropped_writer_queue": self._state_action_samples_dropped_writer_queue,
                "scheduler_missed_deadlines": self._state_action_scheduler_missed_deadlines,
                "write_errors": self._state_action_write_errors,
                "achieved_rate_hz": (
                    self._state_action_samples_written / duration_sec
                    if duration_sec > 0.0 else 0.0
                ),
            }
            writer_stats = {
                "auxiliary_messages_dropped_writer_queue": (
                    self._auxiliary_messages_dropped_writer_queue),
                "state_action_queue_depth": self.state_action_write_queue_depth,
                "auxiliary_queue_depth": self.auxiliary_write_queue_depth,
                "image_queue_depth": self.image_write_queue_depth,
            }
            writer = self._writer
            self._writer_accepting_generation = 0
            self._writer = None
            self._episode_dir = None
            self._episode_name = ""
            self._episode_task = ""
            self._episode_started_ns = 0
            self._episode_started_steady_ns = 0

        # Let rosbag2 finalize metadata before changing/removing the directory.
        del writer

        camera_summary = ", ".join(
            f"{camera_name}:recv={stats['received']},enc={stats['encoded']},"
            f"write={stats['written']},drop_rate={stats['dropped_rate_limit']},"
            f"drop_q={stats['dropped_queue']},"
            f"drop_writer={stats['dropped_writer_queue']},"
            f"drop_stale={stats['dropped_stale']},"
            f"errors={stats['encode_errors'] + stats['write_errors']}"
            for camera_name, stats in camera_stats.items()
        )
        self.get_logger().info(
            f"Episode RGB summary ({duration_sec:.3f}s): {camera_summary}")
        self.get_logger().info(
            "Episode right-arm state/action summary: "
            f"written={state_action_stats['samples_written']}, "
            f"skipped={state_action_stats['samples_skipped_stale_or_missing']}, "
            f"achieved={state_action_stats['achieved_rate_hz']:.2f} Hz")
        zero_frame_cameras = [
            camera_name
            for camera_name, stats in camera_stats.items()
            if stats["written"] == 0
        ]
        if save and zero_frame_cameras:
            self.get_logger().error(
                "Successful episode contains zero written RGB frames for: "
                + ", ".join(zero_frame_cameras)
                + "; saving is still allowed because camera frame-rate validation is disabled"
            )

        if save:
            try:
                self._update_session_result(
                    episode_dir,
                    "success",
                    True,
                    duration_sec=duration_sec,
                    camera_stats=camera_stats,
                    state_action_stats=state_action_stats,
                    writer_stats=writer_stats,
                )
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
        receive_time_ns = time.monotonic_ns()
        positions = list(message.joint_states.position)
        velocities = list(message.joint_states.velocity)
        if len(positions) < 26:
            return
        arm_positions = positions[19:26]
        arm_velocities = velocities[19:26] if len(velocities) >= 26 else None
        derived = self._arm_joint_state(
            message.stamp, self.joint_names, arm_positions, arm_velocities)
        with self._lock:
            self._latest_observation = derived
            self._latest_observation_received_ns = receive_time_ns

    def _command_callback(self, message):
        receive_time_ns = time.monotonic_ns()
        commands = list(message.commands)
        if len(commands) < 26:
            return
        arm_positions = [command.pos for command in commands[19:26]]
        derived = self._arm_joint_state(
            message.stamp, self.joint_names, arm_positions)
        with self._lock:
            self._latest_arm_action = derived
            self._latest_arm_action_received_ns = receive_time_ns

    def _o6_trigger_callback(self, message):
        receive_time_ns = time.monotonic_ns()
        with self._lock:
            # Bit 1 is the right O6 command state: 0=open, 1=closed.
            self._latest_right_gripper_closed = 1 if (int(message.data) & 0x02) else 0
            self._latest_gripper_received_ns = receive_time_ns

    def _sample_state_action_at(self, generation, sample_ros_ns, sample_steady_ns):
        warning = None

        with self._lock:
            if not self._active or self._episode_generation != generation:
                return

            sources = {
                "observation": (
                    self._latest_observation,
                    self._latest_observation_received_ns,
                ),
                "arm_action": (
                    self._latest_arm_action,
                    self._latest_arm_action_received_ns,
                ),
                "right_gripper_action": (
                    self._latest_right_gripper_closed,
                    self._latest_gripper_received_ns,
                ),
            }
            stale_sources = []
            for source_name, (value, received_ns) in sources.items():
                if value is None or received_ns <= 0:
                    stale_sources.append(f"{source_name}=missing")
                    continue
                age_ns = sample_steady_ns - received_ns
                if age_ns > self.state_action_timeout_ns:
                    stale_sources.append(f"{source_name}={age_ns / 1e9:.3f}s")

            if stale_sources:
                self._state_action_samples_skipped += 1
                skipped = self._state_action_samples_skipped
                if skipped == 1 or skipped % 100 == 0:
                    warning = (
                        f"Skipping right-arm {self.state_action_rate_hz:.1f} Hz sample "
                        "because source data is stale: "
                        + ", ".join(stale_sources)
                        + f" (skipped={skipped})"
                    )
            else:
                observation = copy.deepcopy(self._latest_observation)
                arm_action = copy.deepcopy(self._latest_arm_action)
                sample_sec, sample_nanosec = divmod(int(sample_ros_ns), 1_000_000_000)
                observation.header.stamp.sec = int(sample_sec)
                observation.header.stamp.nanosec = int(sample_nanosec)
                arm_action.header.stamp.sec = int(sample_sec)
                arm_action.header.stamp.nanosec = int(sample_nanosec)
                gripper_action = UInt8()
                gripper_action.data = int(self._latest_right_gripper_closed)
                batch = WriteBatch(
                    generation=generation,
                    timestamp_ns=int(sample_ros_ns),
                    messages=(
                        (self.observation_joint_topic, observation),
                        (self.action_joint_topic, arm_action),
                        (self.right_gripper_action_topic, gripper_action),
                    ),
                    kind="state_action",
                )
                if self._enqueue_write_batch_locked(
                        self._state_action_write_queue, batch):
                    self._state_action_samples_enqueued += 1

        if warning is not None:
            self.get_logger().warning(warning)

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
                self._enqueue_message_locked(self.joy_topic, message, receive_time_ns)

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
                    self._enqueue_message_locked(
                        self.joy_topic, message, self.get_clock().now().nanoseconds)

    def _image_callback(self, camera_name, message):
        receive_time_ns = self.get_clock().now().nanoseconds
        receive_steady_ns = time.monotonic_ns()
        with self._lock:
            self._camera_last_seen_ns[camera_name] = receive_steady_ns
            self._camera_last_input_shape[camera_name] = (
                int(message.width), int(message.height))
            if not self._active:
                return
            episode_generation = self._episode_generation
            stats = self._camera_stats[camera_name]
            stats["received"] += 1
            if self._camera_rate_limit_enabled[camera_name]:
                output_period_ns = self._camera_output_period_ns[camera_name]
                elapsed_ns = max(0, receive_steady_ns - self._episode_started_steady_ns)
                output_slot = elapsed_ns // output_period_ns
                if output_slot <= self._camera_last_output_slot[camera_name]:
                    stats["dropped_rate_limit"] += 1
                    return
                self._camera_last_output_slot[camera_name] = output_slot

        item = (episode_generation, message, receive_time_ns)
        image_queue = self._image_queues[camera_name]
        try:
            image_queue.put_nowait(item)
            with self._lock:
                if episode_generation == self._episode_generation:
                    self._camera_stats[camera_name]["enqueued"] += 1
            return
        except Full:
            pass

        try:
            image_queue.get_nowait()
            image_queue.task_done()
            with self._lock:
                if episode_generation == self._episode_generation:
                    self._camera_stats[camera_name]["dropped_queue"] += 1
        except Empty:
            pass

        try:
            image_queue.put_nowait(item)
            with self._lock:
                if episode_generation == self._episode_generation:
                    self._camera_stats[camera_name]["enqueued"] += 1
        except Full:
            with self._lock:
                if episode_generation == self._episode_generation:
                    self._camera_stats[camera_name]["dropped_queue"] += 1

    def _topic_callback(self, topic, message):
        with self._lock:
            if self._active:
                self._enqueue_message_locked(
                    topic, message, self.get_clock().now().nanoseconds)

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
    executor = None
    try:
        node = EpisodeRecorder()
        executor = MultiThreadedExecutor(num_threads=node.executor_threads)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.stop_for_shutdown()
            node._stop_state_sampler()
            node._stop_camera_workers()
            node._stop_writer_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
