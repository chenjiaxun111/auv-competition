#!/usr/bin/env python3
"""Standalone ROS 2 subscriber for AUV SLAM handoff topics.

Run after sourcing a ROS 2 Humble environment:
  export ROS_DOMAIN_ID=0
  python3 subscribe_auv_slam_topics.py
"""

import argparse
import math
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32, Float32MultiArray

"""Standalone ROS 2 subscriber for AUV SLAM handoff topics.

Run after sourcing a ROS 2 Humble environment:
  export ROS_DOMAIN_ID=0
  python3 subscribe_auv_slam_topics.py
"""
from tf2_msgs.msg import TFMessage


def qos_reliable(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def qos_best_effort(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def qos_transient_local(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def stamp_to_float(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def summarize_odom(msg: Odometry) -> str:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (
        f"frame={msg.header.frame_id} child={msg.child_frame_id} "
        f"pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) yaw={yaw:.2f} rad"
    )


def summarize_grid(msg: OccupancyGrid) -> str:
    return (
        f"frame={msg.header.frame_id} "
        f"size={msg.info.width}x{msg.info.height} "
        f"resolution={msg.info.resolution:.3f}"
    )


def summarize_tf(msg: TFMessage) -> str:
    if not msg.transforms:
        return "transforms=0"
    frames = [f"{t.header.frame_id}->{t.child_frame_id}" for t in msg.transforms[:3]]
    suffix = "" if len(msg.transforms) <= 3 else f" ... +{len(msg.transforms) - 3}"
    return f"transforms={len(msg.transforms)} [{', '.join(frames)}{suffix}]"


def summarize_clock(msg: Clock) -> str:
    return f"sim_time={msg.clock.sec}.{msg.clock.nanosec:09d}"


def summarize_image(msg: Image) -> str:
    return f"frame={msg.header.frame_id} {msg.width}x{msg.height} encoding={msg.encoding}"


def summarize_imu(msg: Imu) -> str:
    a = msg.linear_acceleration
    w = msg.angular_velocity
    return f"frame={msg.header.frame_id} accel=({a.x:.2f},{a.y:.2f},{a.z:.2f}) gyro=({w.x:.2f},{w.y:.2f},{w.z:.2f})"


def summarize_camera_info(msg: CameraInfo) -> str:
    return f"frame={msg.header.frame_id} {msg.width}x{msg.height}"


def summarize_float32(msg: Float32) -> str:
    return f"value={msg.data:.3f}"


def summarize_float_array(msg: Float32MultiArray) -> str:
    preview = ", ".join(f"{v:.3f}" for v in msg.data[:6])
    suffix = "" if len(msg.data) <= 6 else f", ... +{len(msg.data) - 6}"
    return f"len={len(msg.data)} [{preview}{suffix}]"


def summarize_twist(msg: TwistStamped) -> str:
    v = msg.twist.linear
    w = msg.twist.angular
    return f"frame={msg.header.frame_id} linear=({v.x:.2f},{v.y:.2f},{v.z:.2f}) angular=({w.x:.2f},{w.y:.2f},{w.z:.2f})"


@dataclass
class TopicState:
    msg_type_name: str
    count: int = 0
    last_count: int = 0
    first_wall_time: Optional[float] = None
    last_wall_time: Optional[float] = None
    last_summary: str = ""


class AuvSlamSubscriber(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("auv_slam_topic_subscriber")
        self.status_period = args.status_period
        self.states: Dict[str, TopicState] = {}

        self._add(Odometry, args.odom_topic, qos_reliable(), summarize_odom)
        self._add(Odometry, "/auv/camera/odom", qos_reliable(), summarize_odom)
        self._add(Clock, "/clock", qos_best_effort(), summarize_clock)
        self._add(TFMessage, "/tf", qos_reliable(depth=100), summarize_tf)
        self._add(TFMessage, "/tf_static", qos_transient_local(depth=100), summarize_tf)
        self._add(OccupancyGrid, "/rtabmap/octomap_grid", qos_transient_local(), summarize_grid)
        self._add(OccupancyGrid, "/rtabmap/map", qos_transient_local(), summarize_grid)

        if args.with_sensors:
            self._add(Image, "/auv/camera/image_raw", qos_profile_sensor_data, summarize_image)
            self._add(Image, "/auv/camera/left/color/rect/image", qos_profile_sensor_data, summarize_image)
            self._add(Image, "/auv/camera/right/color/rect/image", qos_profile_sensor_data, summarize_image)
            self._add(Imu, "/auv/camera/imu/data", qos_profile_sensor_data, summarize_imu)
            self._add(CameraInfo, "/auv/camera/left/camera_info", qos_reliable(), summarize_camera_info)
            self._add(CameraInfo, "/auv/camera/right/camera_info", qos_reliable(), summarize_camera_info)
            self._add(Float32, "/auv/depth", qos_reliable(), summarize_float32)
            self._add(Float32MultiArray, "/auv/dvl/ranges", qos_reliable(), summarize_float_array)
            self._add(TwistStamped, "/auv/dvl/twist", qos_reliable(), summarize_twist)

        self.create_timer(self.status_period, self._print_status)
        self.get_logger().info("Subscriber started. Waiting for messages...")
        self.get_logger().info("Use Ctrl-C to stop.")

    def _add(self, msg_type, topic: str, qos: QoSProfile, summarize: Callable) -> None:
        self.states[topic] = TopicState(msg_type_name=f"{msg_type.__module__}.{msg_type.__name__}")

        def callback(msg, topic_name=topic, summarize_fn=summarize):
            now = self.get_clock().now().nanoseconds * 1e-9
            state = self.states[topic_name]
            state.count += 1
            state.last_wall_time = now
            state.last_summary = summarize_fn(msg)
            if state.first_wall_time is None:
                state.first_wall_time = now
                self.get_logger().info(f"[FIRST] {topic_name}: {state.last_summary}")

        self.create_subscription(msg_type, topic, callback, qos)

    def _print_status(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        lines = ["Topic status:"]
        for topic, state in self.states.items():
            delta = state.count - state.last_count
            state.last_count = state.count
            hz = delta / self.status_period
            if state.last_wall_time is None:
                status = "WAIT"
                age_text = "never"
            else:
                age = now - state.last_wall_time
                status = "OK" if age < self.status_period * 2.5 else "STALE"
                age_text = f"{age:.1f}s ago"
            lines.append(
                f"  {status:5s} {topic:38s} count={state.count:<6d} "
                f"rate={hz:6.2f} Hz last={age_text}"
            )
        self.get_logger().info("\n".join(lines))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to AUV SLAM handoff topics and print received status."
    )
    parser.add_argument(
        "--odom-topic",
        default="/auv/odom",
        help="Main odometry topic to subscribe. Default: /auv/odom",
    )
    parser.add_argument(
        "--with-sensors",
        action="store_true",
        help="Also subscribe to camera, IMU, depth and DVL topics.",
    )
    parser.add_argument(
        "--status-period",
        type=float,
        default=5.0,
        help="Seconds between status summaries. Default: 5.0",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = AuvSlamSubscriber(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()