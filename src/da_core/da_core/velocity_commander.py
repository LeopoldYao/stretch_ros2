#!/usr/bin/env python3

from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Vector3


class RealVelocityCommander(Node):
    def __init__(self):
        super().__init__('real_velocity_commander')
        self.pub = self.create_publisher(Float64MultiArray, 'stretch_controller/joint_cmd', 10)

    def pub_vel(self, q_dot: Sequence[float]):
        assert len(q_dot) == 8
        self.get_logger().debug(f"Velocity commander: {q_dot}")
        self.pub.publish(Float64MultiArray(data=q_dot))

    def stop(self):
        stop_msg = Float64MultiArray(data=[0] * 8)
        self.pub.publish(stop_msg)


class SimVelocityCommander(Node):
    def __init__(self):
        super().__init__('sim_velocity_commander')
        self.arm_vel_pub = self.create_publisher(Float64MultiArray, 'stretch_arm_controller/command', 10)
        self.base_vel_pub = self.create_publisher(Twist, 'stretch_diff_drive_controller/cmd_vel', 10)

    def pub_vel(self, q_dot: Sequence[float]):
        arm_msg = Float64MultiArray(data=q_dot[2:])
        self.arm_vel_pub.publish(arm_msg)

        base_msg = Twist()
        base_msg.linear.x = q_dot[0]
        base_msg.angular.z = q_dot[1]
        self.base_vel_pub.publish(base_msg)

    def stop(self):
        arm_stop_msg = Float64MultiArray(data=[0] * 6)
        base_stop_msg = Twist()
        base_stop_msg.linear.x = 0
        base_stop_msg.angular.z = 0
        r = rclpy.create_rate(10)
        for _ in range(3):
            self.arm_vel_pub.publish(arm_stop_msg)
            self.base_vel_pub.publish(base_stop_msg)
            r.sleep()

def main(args=None):
    rclpy.init(args=args)
    sim_velocity_commander = SimVelocityCommander()
    rclpy.spin(sim_velocity_commander)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
