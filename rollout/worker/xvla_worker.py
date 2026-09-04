#!/usr/bin/env python3
"""Python 3.12 XVLA inference worker for the host-side rollout bridge.

No ROS package is imported here.  The worker obtains the newest RGB/q
observation from localhost, runs the exact pre/postprocessors saved beside the
checkpoint, and returns the first configured action steps of the XVLA chunk.
"""

from __future__ import annotations

import argparse
from multiprocessing.connection import Client
from pathlib import Path
import os
import time
import traceback
from typing import Any

import numpy as np
import torch
import yaml


def as_numpy(value: Any) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


class XVLAWorker:
    def __init__(self, config_path: Path) -> None:
        with config_path.open("r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream) or {}
        environment = self.config["environment"]
        os.environ.setdefault("HF_HOME", str(environment["hf_home"]))
        os.environ.setdefault("CUDA_VISIBLE_DEVICES", str(environment["cuda_visible_devices"]))
        model_config = self.config["model"]
        self.checkpoint = Path(model_config["checkpoint"]).expanduser().resolve()
        if not (self.checkpoint / "config.json").is_file():
            raise RuntimeError(f"checkpoint is not a LeRobot pretrained_model directory: {self.checkpoint}")
        self.task = str(model_config["task"]).strip()
        if not self.task:
            raise RuntimeError("model.task must be non-empty")
        self.action_steps = max(1, int(model_config["action_steps_to_send"]))
        ipc = self.config["ipc"]
        self.address = (str(ipc["host"]), int(ipc["port"]))
        if self.address[0] not in {"127.0.0.1", "localhost", "::1"}:
            raise RuntimeError("rollout IPC must remain localhost-only")
        self.authkey = str(ipc["authkey"]).encode("utf-8")
        self.retry_delay = max(0.1, float(ipc["retry_delay_sec"]))
        self.not_ready_delay = max(0.005, float(ipc["not_ready_delay_sec"]))

        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.xvla.modeling_xvla import XVLAPolicy

        print(f"Loading XVLA checkpoint: {self.checkpoint}", flush=True)
        self.policy = XVLAPolicy.from_pretrained(
            pretrained_name_or_path=self.checkpoint,
            strict=False,
            local_files_only=True,
        )
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config, pretrained_path=self.checkpoint)
        self.policy.reset()
        print(
            f"XVLA worker ready: device={self.policy.config.device}, "
            f"chunk={self.policy.config.chunk_size}, send_steps={self.action_steps}",
            flush=True)

    def infer(self, observation: dict[str, Any]) -> tuple[np.ndarray, float]:
        images = observation["images"]
        raw_observation = {
            # Keep source names: saved checkpoint preprocessor performs the training rename map.
            "observation.images.head": np.ascontiguousarray(images["head"]),
            "observation.images.left": np.ascontiguousarray(images["left"]),
            "observation.images.right": np.ascontiguousarray(images["right"]),
            # Training observation.state is right-arm q(7), not velocity or action.
            "observation.state": np.asarray(observation["right_q"], dtype=np.float32),
            "task": self.task,
        }
        start = time.perf_counter()
        batch = self.preprocessor(raw_observation)
        with torch.inference_mode():
            chunk = self.policy.predict_action_chunk(batch)
        steps = min(self.action_steps, int(chunk.shape[1]))
        actions: list[np.ndarray] = []
        for index in range(steps):
            action = self.postprocessor(chunk[:, index])
            array = as_numpy(action)
            array = np.asarray(array, dtype=np.float32).reshape(-1)
            if array.size != 8 or not np.all(np.isfinite(array)):
                raise RuntimeError(f"invalid postprocessed XVLA action at step {index}: shape={array.shape}")
            actions.append(array)
        return np.stack(actions, axis=0), (time.perf_counter() - start) * 1000.0

    def run(self) -> None:
        while True:
            connection = None
            try:
                connection = Client(self.address, authkey=self.authkey)
                print(f"Connected to rollout ROS bridge at {self.address}", flush=True)
                while True:
                    connection.send({"type": "next_observation"})
                    reply = connection.recv()
                    if reply.get("type") != "observation":
                        raise RuntimeError(f"unexpected IPC reply: {reply}")
                    if reply.get("status") == "finished":
                        print("Rollout bridge reported completion; XVLA worker exits.", flush=True)
                        return
                    if reply.get("status") != "ready":
                        time.sleep(self.not_ready_delay)
                        continue
                    try:
                        actions, latency_ms = self.infer(reply)
                        connection.send({
                            "type": "action_chunk",
                            "actions": actions,
                            "inference_latency_ms": latency_ms,
                        })
                        ack = connection.recv()
                        if not ack.get("ok", False):
                            print(f"Bridge rejected action: {ack}", flush=True)
                        else:
                            print(
                                f"XVLA action chunk accepted: {ack.get('accepted')} steps, "
                                f"inference={latency_ms:.1f} ms", flush=True)
                    except Exception as error:
                        detail = "".join(traceback.format_exception_only(type(error), error)).strip()
                        print(f"XVLA inference error: {detail}", flush=True)
                        connection.send({"type": "worker_error", "reason": detail})
                        connection.recv()
                        time.sleep(self.retry_delay)
            except KeyboardInterrupt:
                return
            except (ConnectionError, EOFError, OSError, RuntimeError) as error:
                print(f"XVLA worker IPC error: {error}; reconnecting in {self.retry_delay:.1f}s", flush=True)
                time.sleep(self.retry_delay)
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except OSError:
                        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="/home/ub/program/qiling_television/rollout/config/xvla_rollout.yaml",
        help="Path to xvla_rollout.yaml",
    )
    args = parser.parse_args()
    XVLAWorker(Path(args.config).expanduser().resolve()).run()


if __name__ == "__main__":
    main()
