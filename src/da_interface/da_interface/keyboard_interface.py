#!/usr/bin/env python3

import threading
import sys
import select
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


msg = """
---------------------------
Moving around:
       w
    a  s  d

Moving arm:
       i
    j  k  l

Turning gripper:
    u     o

anything else : stop

z/x : increase/decrease only linear speed by 10%
c/v : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    "w": (1, 0, 0, 0, 0, 0),
    "s": (-1, 0, 0, 0, 0, 0),
    "a": (0, 1, 0, 0, 0, 0),
    "d": (0, -1, 0, 0, 0, 0),
    "i": (0, 0, 1, 0, 0, 0),
    "k": (0, 0, -1, 0, 0, 0),
    "j": (0, 0, 0, -1, 0, 0),
    "l": (0, 0, 0, 1, 0, 0),
    "u": (0, 0, 0, 0, 1, 0),
    "o": (0, 0, 0, 0, -1, 0),
    "f": (0, 0, 0, 0, 0, 1),
    "r": (0, 0, 0, 0, 0, -1),
}

speedBindings = {
    "z": (1.1, 1),
    "x": (0.9, 1),
    "c": (1, 1.1),
    "v": (1, 0.9),
}


class PublishThread(threading.Thread):
    def __init__(self, rate, node):
        super(PublishThread, self).__init__()
        self.publisher = node.create_publisher(Float64MultiArray, "/teleop_velocity_command", 1)
        self.gripper_publisher = node.create_publisher(Float64, "/stretch_controller/gripper_cmd", 1)
        self.x = 0.0
        self.th = 0.0
        self.lift = 0.0
        self.arm_ext = 0.0
        self.wrist = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.gripper = 0.0
        self.condition = threading.Condition()
        self.done = False

        self.timeout = 1.0 / rate if rate != 0.0 else None
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to topic")
            time.sleep(0.5)
            i += 1
            i %= 5
        if not rclpy.ok():
            raise RuntimeError("Got shutdown request before subscribers connected")

    def update(self, x, th, lift, arm_ext, wrist, gripper, speed, turn):
        self.condition.acquire()
        self.x = x
        self.th = th
        self.lift = lift
        self.arm_ext = arm_ext
        self.wrist = wrist
        self.gripper = gripper
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            x = self.x * self.speed
            th = self.th * self.turn
            lift = self.lift * self.speed
            single_arm_ext = self.arm_ext * self.speed / 4
            wrist = self.wrist * self.turn
            gripper = self.gripper * self.speed

            self.condition.release()

            self.publisher.publish(Float64MultiArray(data=[x, th, lift, *[single_arm_ext] * 4, wrist]))
            self.gripper_publisher.publish(Float64(data=gripper))

        self.publisher.publish(Float64MultiArray(data=[0] * 8))
        self.gripper_publisher.publish(Float64(data=0))


def getKey(settings, timeout):
    if sys.platform == "win32":
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return None if sys.platform == "win32" else termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_interface')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('speed_limit', 1.0)
        self.declare_parameter('turn_limit', 1.0)
        self.declare_parameter('repeat_rate', 0.0)
        self.declare_parameter('key_timeout', 0.1)
        self.declare_parameter('frame_id', '')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    settings = saveTerminalSettings()

    speed = node.get_parameter('speed').get_parameter_value().double_value
    turn = node.get_parameter('turn').get_parameter_value().double_value
    speed_limit = node.get_parameter('speed_limit').get_parameter_value().double_value
    turn_limit = node.get_parameter('turn_limit').get_parameter_value().double_value
    repeat = node.get_parameter('repeat_rate').get_parameter_value().double_value
    key_timeout = node.get_parameter('key_timeout').get_parameter_value().double_value

    pub_thread = PublishThread(repeat, node)

    x = 0
    lift = 0
    arm_ext = 0
    wrist = 0
    th = 0.0
    gripper = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, th, lift, arm_ext, wrist, gripper, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                lift = moveBindings[key][2]
                arm_ext = moveBindings[key][3]
                wrist = moveBindings[key][4]
                gripper = moveBindings[key][5]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                speed = min(speed_limit, speed)
                turn = min(turn_limit, turn)
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                if key == "" and x == 0 and th == 0 and lift == 0 and arm_ext == 0 and wrist == 0:
                    continue
                x, th, lift, arm_ext, wrist, gripper = [0] * 6
                if key == "\x03":
                    break

            pub_thread.update(x, th, lift, arm_ext, wrist, gripper, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
