#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TeleopKeyboard:
    def __init__(self):
        rospy.init_node('teleop_keyboard')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_mapping = {
            'w': (1.0, 0.0),
            's': (-1.0, 0.0),
            'a': (0.0, 1.0),
            'd': (0.0, -1.0),
            'q': (0.0, 0.0)  # Stop
        }
        self.run()

    def run(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key in self.key_mapping:
                        linear, angular = self.key_mapping[key]
                        twist = Twist()
                        twist.linear.x = linear
                        twist.angular.z = angular
                        self.pub.publish(twist)
                    elif key == '\x03':  # Ctrl+C
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    TeleopKeyboard()

