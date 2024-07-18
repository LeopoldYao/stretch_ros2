from __future__ import annotations

import abc
from typing import Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from .velocity_commander import RealVelocityCommander, SimVelocityCommander


class ControllerBase(Node, abc.ABC):
    def __init__(self, node_name: str, uh_topic: str, verbose=False) -> None:
        super().__init__(node_name)
        self.verbose = verbose
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.use_real_robot = self.declare_parameter("use_real_robot", False).get_parameter_value().bool_value
        self._vel_cmder = (
            RealVelocityCommander()
            if self.use_real_robot
            else SimVelocityCommander()
        )
        if uh_topic:
            self.create_subscription(Float64MultiArray, uh_topic, self.uh_cb, 10)
            self.uh = np.zeros(8)
        self.max_speed = None

    def clamp_qd(self, q_dot: np.ndarray) -> np.ndarray:
        if self.max_speed is None:
            return q_dot
        length = np.linalg.norm(q_dot)
        if length > self.max_speed:
            self.get_logger().info(f"Clamping velocity: {length}")
            q_dot = q_dot / length * self.max_speed
        return q_dot

    def uh_cb(self, msg: Float64MultiArray) -> None:
        """Human input callback function."""
        self.uh = np.array(msg.data)
        assert len(self.uh) == 8, "Human input should be 8-dimensional"
        if self.verbose:
            self.get_logger().info(f"Received: {self.uh}")

    @abc.abstractmethod
    def step(self):
        raise NotImplementedError()

    def get_err(self, target_frame="goal", base_frame="base_link") -> np.ndarray | None:
        try:
            now = self.get_clock().now().to_msg()
            target_transform: TransformStamped = self._tf_buffer.lookup_transform(
                base_frame, target_frame, now, timeout=rclpy.duration.Duration(seconds=5)
            )
            target_pos = target_transform.transform.translation
            ee_transform: TransformStamped = self._tf_buffer.lookup_transform(
                base_frame, "link_grasp_center", now, timeout=rclpy.duration.Duration(seconds=5)
            )
            ee_pos = ee_transform.transform.translation
            return np.array(
                [target_pos.x - ee_pos.x, target_pos.y - ee_pos.y, target_pos.z - ee_pos.z, 0, 0, 0]
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Transform error: {str(e)}')
            return None

    def publish_vel(self, q_dot: Sequence[float]) -> None:
        self._vel_cmder.pub_vel(q_dot)

    def stop(self) -> None:
        self._vel_cmder.stop()
        self.destroy_node()


