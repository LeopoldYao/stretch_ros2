#!/usr/bin/env python3

"""
Controller which directly passes the velocity provided by the interface to stretch
author: Chen Chen
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from .controller_base import ControllerBase


class TeleopController(ControllerBase):
    def __init__(self ,uh_topic="teleop_velocity_command") -> None:
        super().__init__(node_name = 'teleop_controller' , uh_topic = uh_topic)

    def step(self):
        self.get_logger().debug(f"Teleop controller sends: {self.uh}")
        self._vel_cmder.pub_vel(self.uh)



def main(args=None):
    rclpy.init(args=args)
    teleop_controller = TeleopController()
    r = teleop_controller.create_rate(15)

    try:
        while rclpy.ok():
            teleop_controller.step()
            r.sleep()
    except KeyboardInterrupt:
        teleop_controller.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        teleop_controller.stop()
        teleop_controller.destroy_node()
        rclpy.shutdown()




if __name__ == "__main__":
    main()


