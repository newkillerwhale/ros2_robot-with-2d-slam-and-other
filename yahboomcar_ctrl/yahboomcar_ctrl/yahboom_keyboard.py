#!/usr/bin/env python
# encoding: utf-8
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

a / d : Y axis + / -
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    # (x, y, th)
    'i': (1, 0, 0),
    'o': (1, 0, -1),
    'j': (0, 0, 1),
    'l': (0, 0, -1),
    'u': (1, 0, 1),
    ',': (-1, 0, 0),
    '.': (-1, 0, 1),
    'm': (-1, 0, -1),
    'I': (1, 0, 0),
    'O': (1, 0, -1),
    'J': (0, 0, 1),
    'L': (0, 0, -1),
    'U': (1, 0, 1),
    'M': (-1, 0, -1),

    # 新增 Y 方向
    'a': (0, 1, 0),
    'A': (0, 1, 0),
    'd': (0, -1, 0),
    'D': (0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
    'Q': (1.1, 1.1),
    'Z': (0.9, 0.9),
    'W': (1.1, 1),
    'X': (0.9, 1),
    'E': (1, 1.1),
    'C': (1, 0.9),
}


class YahboomKeybord(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.declare_parameter('linear_speed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').value
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return 'currently:\tspeed {:.2f}\tturn {:.2f}'.format(speed, turn)


def main():
    rclpy.init()
    node = YahboomKeybord('yahboom_keyboard_ctrl')

    xspeed_switch = True
    speed, turn = 0.2, 0.2         # 默认转向速度降到 0.5
    x, y, th = 0, 0, 0
    status = 0
    stop = False
    count = 0

    twist = Twist()

    try:
        print(msg)
        print(node.vels(speed, turn))
        while True:
            key = node.get_key()

            if key in ('t', 'T'):
                xspeed_switch = not xspeed_switch
            elif key in ('s', 'S'):
                stop = not stop
                print('stop keyboard control: {}'.format(not stop))

            if key in moveBindings:
                x, y, th = moveBindings[key]
                count = 0
            elif key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                count = 0

                speed = min(speed, node.linear_speed_limit)
                turn = min(turn, node.angular_speed_limit)
                print(node.vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k':
                x, y, th = 0, 0, 0
            else:
                count += 1
                if count > 4:
                    x, y, th = 0, 0, 0
                if key == '\x03':  # Ctrl-C
                    break

            if xspeed_switch:
                twist.linear.x = speed * x
                twist.linear.y = speed * y
            else:
                twist.linear.x = speed * y
                twist.linear.y = speed * x
            twist.angular.z = turn * th

            if not stop:
                node.pub.publish(twist)
            else:
                node.pub.publish(Twist())

    except Exception as e:
        print(e)
    finally:
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
