#!/usr/bin/env python3
import json
import math
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def sample_is_due(
    now_ns: int, deadline_ns: Optional[int]
) -> bool:
    """Return whether a callback has reached the next sample deadline."""
    return deadline_ns is None or now_ns >= deadline_ns


def advance_sample_deadline(
    deadline_ns: Optional[int], now_ns: int, period_ns: int
) -> int:
    """Advance a deadline, skipping intervals missed during a stall."""
    if deadline_ns is None:
        return now_ns + period_ns

    next_deadline_ns = deadline_ns + period_ns
    if next_deadline_ns <= now_ns:
        return now_ns + period_ns
    return next_deadline_ns


class FrameRecorder(Node):
    """Records one VLA episode as JPEG frames plus JSON Lines metadata.

    This is an intentionally simple raw format. It is easy to inspect and can
    later be converted into the exact LeRobot version installed for training.
    """

    def __init__(self) -> None:
        super().__init__("frame_recorder")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("action_topic", "/teleop/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("instruction_topic", "/vla/instruction")
        self.declare_parameter("output_root", "~/rover_vla_data/raw")
        self.declare_parameter("episode_id", 0)
        self.declare_parameter("sample_rate_hz", 10.0)
        self.declare_parameter("state_source", "odom")
        self.declare_parameter("state_freshness_timeout_sec", 1.0)
        self.declare_parameter("require_instruction", True)
        self.declare_parameter("jpeg_quality", 92)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.instruction_topic = str(self.get_parameter("instruction_topic").value)
        output_root = Path(str(self.get_parameter("output_root").value)).expanduser()
        self.episode_id = int(self.get_parameter("episode_id").value)
        rate = max(float(self.get_parameter("sample_rate_hz").value), 0.1)
        self.minimum_period = 1.0 / rate
        self.sample_period_ns = max(1, round(1_000_000_000 / rate))
        self.state_source = str(self.get_parameter("state_source").value).strip().lower()
        self.state_freshness_timeout = float(
            self.get_parameter("state_freshness_timeout_sec").value
        )
        self.require_instruction = bool(self.get_parameter("require_instruction").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        if self.state_source not in {"odom", "previous_action"}:
            raise ValueError("state_source must be 'odom' or 'previous_action'")
        if (
            not math.isfinite(self.state_freshness_timeout)
            or self.state_freshness_timeout <= 0.0
        ):
            raise ValueError(
                "state_freshness_timeout_sec must be finite and positive"
            )

        self.episode_dir = output_root / f"episode_{self.episode_id:04d}"
        if self.episode_dir.exists():
            raise FileExistsError(f"Episode directory already exists: {self.episode_dir}")
        self.images_dir = self.episode_dir / "images"
        self.images_dir.mkdir(parents=True)
        self.jsonl_path = self.episode_dir / "frames.jsonl"
        self.jsonl_file = self.jsonl_path.open("w", encoding="utf-8")

        self.bridge = CvBridge()
        self.latest_instruction: Optional[str] = None
        self.latest_action = np.zeros(2, dtype=np.float32)
        self.previous_saved_action = np.zeros(2, dtype=np.float32)
        self.latest_odom = np.zeros(2, dtype=np.float32)
        self.latest_odom_wall_time: Optional[float] = None
        self.have_action = False
        self.have_odom = False
        self.frame_index = 0
        self.next_save_monotonic_ns: Optional[int] = None
        self.start_wall_time = time.time()
        self.first_ros_stamp_ns: Optional[int] = None

        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(Twist, self.action_topic, self.action_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(String, self.instruction_topic, self.instruction_callback, 10)

        self.write_episode_metadata(final=False)
        self.get_logger().info(
            f"Recording episode {self.episode_id} to {self.episode_dir}; "
            f"image={self.image_topic}, action={self.action_topic}, "
            f"state_source={self.state_source}"
        )

    def instruction_callback(self, msg: String) -> None:
        value = msg.data.strip()
        if value:
            self.latest_instruction = value

    def action_callback(self, msg: Twist) -> None:
        values = np.array([msg.linear.x, msg.angular.z], dtype=np.float32)
        if np.all(np.isfinite(values)):
            self.latest_action = values
            self.have_action = True

    def odom_callback(self, msg: Odometry) -> None:
        values = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.angular.z], dtype=np.float32
        )
        if np.all(np.isfinite(values)):
            self.latest_odom = values
            self.latest_odom_wall_time = time.monotonic()
            self.have_odom = True

    @staticmethod
    def stamp_to_ns(msg: Image) -> int:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def ready(self) -> bool:
        if self.require_instruction and not self.latest_instruction:
            return False
        if not self.have_action:
            return False
        if self.state_source == "odom":
            if not self.have_odom or self.latest_odom_wall_time is None:
                return False
            if (
                time.monotonic() - self.latest_odom_wall_time
                > self.state_freshness_timeout
            ):
                return False
        return True

    def image_callback(self, msg: Image) -> None:
        now_monotonic_ns = time.monotonic_ns()
        if not sample_is_due(
            now_monotonic_ns, self.next_save_monotonic_ns
        ):
            return
        if not self.ready():
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        stamp_ns = self.stamp_to_ns(msg)
        if self.first_ros_stamp_ns is None:
            self.first_ros_stamp_ns = stamp_ns
        timestamp_sec = (stamp_ns - self.first_ros_stamp_ns) / 1e9

        if self.state_source == "odom":
            state = self.latest_odom.copy()
        else:
            state = self.previous_saved_action.copy()

        action = self.latest_action.copy()
        filename = f"frame_{self.frame_index:06d}.jpg"
        image_path = self.images_dir / filename

        ok = cv2.imwrite(
            str(image_path),
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().error(f"Failed to save {image_path}")
            return

        record = {
            "episode_index": self.episode_id,
            "frame_index": self.frame_index,
            "timestamp_sec": timestamp_sec,
            "image": f"images/{filename}",
            "instruction": self.latest_instruction or "",
            "state": [float(state[0]), float(state[1])],
            "action": [float(action[0]), float(action[1])],
            "state_source": self.state_source,
            "state_freshness_timeout_sec": self.state_freshness_timeout,
            "image_width": int(bgr.shape[1]),
            "image_height": int(bgr.shape[0]),
        }
        self.jsonl_file.write(json.dumps(record) + "\n")
        self.jsonl_file.flush()

        self.previous_saved_action = action
        self.frame_index += 1
        self.next_save_monotonic_ns = advance_sample_deadline(
            self.next_save_monotonic_ns,
            now_monotonic_ns,
            self.sample_period_ns,
        )

        if self.frame_index % 20 == 0:
            self.get_logger().info(f"Saved {self.frame_index} frames")

    def write_episode_metadata(self, final: bool) -> None:
        metadata = {
            "episode_index": self.episode_id,
            "image_topic": self.image_topic,
            "action_topic": self.action_topic,
            "odom_topic": self.odom_topic,
            "instruction_topic": self.instruction_topic,
            "state_source": self.state_source,
            "state_freshness_timeout_sec": self.state_freshness_timeout,
            "sample_rate_hz": 1.0 / self.minimum_period,
            "num_frames": self.frame_index,
            "instruction": self.latest_instruction,
            "started_unix_sec": self.start_wall_time,
            "finalized": final,
        }
        (self.episode_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )

    def close(self) -> None:
        if not self.jsonl_file.closed:
            self.jsonl_file.flush()
            self.jsonl_file.close()
        self.write_episode_metadata(final=True)
        self.get_logger().info(
            f"Finalized episode {self.episode_id} with {self.frame_index} frames"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FrameRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"frame_recorder failed: {exc}")
        raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
