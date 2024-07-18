#!/usr/bin/env python3

"""
Automatically publish goal marker in gazebo, rviz and tf server
author: Chen Chen
"""

import os

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class GoalMarker(Node):
    def __init__(
        self,
        pub,
        x,
        y,
        z,
        id,
        frame,
        scale=0.1,
        color=(1.0, 1.0, 1.0, 0.8),
        tf_prefix="goal_marker",
        gazebo=False,
        tf=False,
    ):
        super().__init__('goal_marker_node')
        self.pub = pub
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.id = id
        self.frame = frame
        self.scale = float(scale)
        self.color = color
        self.tf_prefix = tf_prefix
        self.gazebo = gazebo
        self.tf = tf
        if self.tf:
            self.br = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_marker()

    def publish_marker(self):
        if self.gazebo:
            self.publish_gazebo_marker()
        self.publish_rviz_marker()
        if self.tf:
            self.broadcast_tf()

    def broadcast_tf(self):
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = self.frame
        tf_stamped.child_frame_id = "{}{}".format(self.tf_prefix, self.id)
        tf_stamped.transform.translation.x = self.x
        tf_stamped.transform.translation.y = self.y
        tf_stamped.transform.translation.z = self.z
        tf_stamped.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.br.sendTransform(tf_stamped)

    def publish_rviz_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame
        marker.id = self.id
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose = Pose(position=Point(x=self.x, y=self.y, z=self.z), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        marker.scale = Vector3(x=self.scale, y=self.scale, z=self.scale)
        marker.color = ColorRGBA(r=self.color[0], g=self.color[1], b=self.color[2], a=self.color[3])
        self.pub.publish(marker)

    def publish_gazebo_marker(self):
        msg = "action: ADD_MODIFY, type: SPHERE, id: {}, ".format(self.id)
        msg += "scale: {{x:{}, y:{}, z:{}}}, ".format(self.scale, self.scale, self.scale)
        msg += "pose: {{position: {{x:{}, y:{}, z:{}}}, orientation: {{x:0, y:0, z:0, w:1}}}}".format(self.x, self.y, self.z)
        os.system("gz marker -m '{}'".format(msg))

    @property
    def position(self):
        return (self.x, self.y, self.z)

    @position.setter
    def position(self, value):
        if len(value) != 3:
            raise AttributeError("position is not 3 dimensional")
        self.x, self.y, self.z = float(value[0]), float(value[1]), float(value[2])

    def update(self, pos=None, color=None):
        if pos is not None:
            self.position = pos
        if color is not None:
            self.color = color
        self.publish_marker()

    def __del__(self):
        # cannot delete tf frame
        if self.gazebo:
            os.system("gz marker -m 'action: DELETE_MARKER, id: {}'".format(self.id))
        marker = Marker(action=Marker.DELETE, id=self.id)
        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('goal_marker_node')
    marker_pub = node.create_publisher(Marker, "/visualization_marker", 2)
    goal_marker = GoalMarker(marker_pub, 1.0, 1.0, 1.0, id=1, frame="odom")

    def on_shutdown():
        goal_marker.__del__()

    rclpy.get_default_context().on_shutdown(on_shutdown)
   
    rclpy.spin(goal_marker)

    goal_marker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()