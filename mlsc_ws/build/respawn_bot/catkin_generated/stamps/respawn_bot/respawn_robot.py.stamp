#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry

# Set your starting and goal positions
start_position = [0, 0, 0]  # Example start position
goal_position = [5, 5, 0]   # Example goal position
threshold = 0.5  # Threshold to consider goal reached

def odom_callback(data):
    global current_position

    # Get the current position of the robot from odometry
    current_position = [data.pose.pose.position.x, data.pose.pose.position.y]

    # Check if the robot is within the threshold of the goal position
    if abs(current_position[0] - goal_position[0]) < threshold and abs(current_position[1] - goal_position[1]) < threshold:
        rospy.loginfo("Goal reached, respawning robot...")
        respawn_robot()

def respawn_robot():
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = 'your_robot_name'  # Replace with your robot's name
        state_msg.pose.position.x = start_position[0]
        state_msg.pose.position.y = start_position[1]
        state_msg.pose.position.z = start_position[2]
        set_state(state_msg)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('robot_respawn', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

