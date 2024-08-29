#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

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
    rate = rospy.Rate(40)  # Increased rate to ensure smooth control

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w':
                twist.linear.x = 20.0  # Increased forward speed
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -5.0  # Increased reverse speed
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 2.0
                twist.angular.z = -5.0 # Increased turn speed
            elif key == 'd':
                twist.linear.x = 2.0
                twist.angular.z = 5.0  # Increased turn speed
            elif key == 'z':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            pub.publish(twist)

            if key == '\x03':  # Ctrl+C to exit
                break

            rate.sleep()

    except Exception as e:
        print(e)

   

