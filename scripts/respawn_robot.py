#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState

rospy.init_node('respawn_robot')

# Start position
start_position = ModelState()
start_position.model_name = 'robot'  # Ensure this matches your robot's model name
start_position.pose.position.x = 0.0
start_position.pose.position.y = 0.0
start_position.pose.position.z = 0.0
start_position.pose.orientation.w = 1.0

# Goal position
goal_position = {'x': -1.630095, 'y': 3.177666, 'z': 0.499997}

def check_and_respawn(event):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = GetModelStateRequest()
        model_state.model_name = 'robot'  # Ensure this matches your robot's model name
        resp = get_model_state(model_state)
        
        current_position = resp.pose.position
        rospy.loginfo(f"Current Position: x={current_position.x}, y={current_position.y}")
        
        if (abs(current_position.x - goal_position['x']) < 0.5 and
                abs(current_position.y - goal_position['y']) < 0.5):
            rospy.loginfo("Goal reached, respawning robot...")
            respawn_robot()
            
    except rospy.ServiceException as e:
        rospy.logerr("Failed to get model state: %s" % e)

def respawn_robot():
    rospy.loginfo("Respawning in 3 seconds...")
    rospy.sleep(3)
    
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = SetModelStateRequest()
        model_state.model_state = start_position
        
        set_model_state(model_state)
        rospy.loginfo("Robot respawned to start position")
        
    except rospy.ServiceException as e:
        rospy.logerr("Failed to respawn robot: %s" % e)

# Timer to periodically check the robot's position
rospy.Timer(rospy.Duration(5), check_and_respawn)  # Adjust the duration as needed

rospy.spin()
