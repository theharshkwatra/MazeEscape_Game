#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

def get_robot_position(model_name='robot'):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_model_state(model_name, 'world')
        return resp.pose.position
    except rospy.ServiceException as e:
        rospy.logerr("GetModelState service call failed: %s", e)
        return None

def set_robot_position(pose, model_name='robot'):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = pose
        model_state.reference_frame = 'world'
        resp = set_model_state(model_state)
        if resp.success:
            rospy.loginfo("Successfully respawned robot at start position.")
        else:
            rospy.logwarn("Failed to respawn robot: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("SetModelState service call failed: %s", e)

def monitor_and_respawn():
    rospy.init_node('robot_respawner', anonymous=True)
    
    # Start position
    start_pose = Pose()
    start_pose.position.x = 0.0  
    start_pose.position.y = 0.0  
    start_pose.position.z = 0.0  
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    # Goal positions
    goal_positions = [
        {'x': 8.995692, 'y': -8.912122, 'z': 0.499997},  # Original goal position
        {'x': -8.997778, 'y': -8.475602, 'z': 0.499997}  # New goal position
    ]
    position_tolerance = 1.0  

    rate = rospy.Rate(1)  
    while not rospy.is_shutdown():
        current_position = get_robot_position()
        if current_position:
            current_x = current_position.x
            current_y = current_position.y
            current_z = current_position.z

            rospy.loginfo("Current Position: x=%.2f, y=%.2f, z=%.2f",
                          current_x, current_y, current_z)

            should_respawn = False
            for goal_position in goal_positions:
                distance_to_goal = ((current_x - goal_position['x'])**2 + 
                                    (current_y - goal_position['y'])**2 +
                                    (current_z - goal_position['z'])**2) ** 0.5

                rospy.loginfo("Distance to goal (x=%.2f, y=%.2f, z=%.2f): %.2f",
                              goal_position['x'], goal_position['y'], goal_position['z'], distance_to_goal)
                
                if distance_to_goal <= position_tolerance:
                    should_respawn = True
                    rospy.loginfo("Reached goal position: x=%.2f, y=%.2f, z=%.2f", 
                                  goal_position['x'], goal_position['y'], goal_position['z'])
                    break  # Exit the loop after finding the goal

            if should_respawn:
                rospy.loginfo("Respawning robot...")
                set_robot_position(start_pose)

        rate.sleep()

if __name__ == '__main__':
    try:
        monitor_and_respawn()
    except rospy.ROSInterruptException:
        pass

