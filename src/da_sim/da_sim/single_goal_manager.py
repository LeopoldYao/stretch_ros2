#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
import tf2_ros
from math import sqrt
from geometry_msgs.msg import TransformStamped
from da_sim.goal_marker import GoalMarker
from visualization_msgs.msg import Marker
import time

def dist_between_tfs(buffer, tf1, tf2):
    try:
        target_transform = buffer.lookup_transform(
            tf1, tf2, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5)
        )
        pos = target_transform.transform.translation
        return sqrt(pos.x**2 + pos.y**2 + pos.z**2)
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        rclpy.logging.get_logger('goal_manager').error(f"Transform lookup failed: {e}")
        return None

def get_new_marker_pose(buffer, range=0.4):
    target_transform = buffer.lookup_transform(
        "odom", "base_link", rclpy.time.Time()
    )
    pos = target_transform.transform.translation
    x = random.choice((-1, 1)) * random.uniform(0.2, range)
    y = random.choice((-1, 1)) * random.uniform(0.2, range)
    z = random.uniform(0.1, 1)
    return (x + pos.x, y + pos.y, z + pos.z)

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 2)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_id = 0
        self.frame = "odom"

        self.wait_for_transform("odom", "base_link")

        x, y, z = get_new_marker_pose(self.tf_buffer)
        self.marker_list = [
            GoalMarker(self.marker_pub, x, y, z, self.goal_id, self.frame, tf_prefix="goal", tf=True)
        ]

        self.create_timer(1.0 / 5.0, self.timer_callback)

    def wait_for_transform(self, target_frame, source_frame):
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
                print(transform)
                self.get_logger().info(f"Transform {target_frame} -> {source_frame} is now available")
                break
            except tf2_ros.LookupException as e:
                self.get_logger().info(f"Waiting for transform {target_frame} -> {source_frame}: {e}")
                rclpy.spin_once(self, timeout_sec=1.0)

    def clear_markers(self):
        del self.marker_list[:]

    def timer_callback(self):
        dist = dist_between_tfs(self.tf_buffer, f"goal{self.goal_id}", "link_grasp_center")
        if dist is not None:
            self.get_logger().info(f"distance: {dist}")
            if dist < 0.1:
                self.marker_list[:] = []
                x, y, z = get_new_marker_pose(self.tf_buffer)
                self.marker_list.append(
                    GoalMarker(self.marker_pub, x, y, z, self.goal_id, self.frame, tf_prefix="goal", tf=True)
                )
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    goal_manager = GoalManager()

    rclpy.get_default_context().on_shutdown(goal_manager.clear_markers)

    try:
        rclpy.spin(goal_manager)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        goal_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
