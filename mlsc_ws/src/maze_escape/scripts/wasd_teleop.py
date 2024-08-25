#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Function to get key presses
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    twist = Twist()

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w':
                twist.linear.x = 2.5
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -1.0
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.5
                twist.angular.z = -0.8
            elif key == 'd':
                twist.linear.x = 0.5
                twist.angular.z = 0.8
            elif key == 'z':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            pub.publish(twist)

            if key == '\x03':  # Ctrl+C to exit
                break

    except Exception as e:
        print(e)

    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

