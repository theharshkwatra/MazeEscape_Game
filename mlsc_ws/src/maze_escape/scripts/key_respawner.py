#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from pynput import keyboard
import threading

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

def on_press(key):
    try:
        if key.char == 'r':
            rospy.loginfo("Received 'r' key press. Respawning robot...")
            set_robot_position(start_pose)
    except AttributeError:
        pass

def listen_for_keys():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

def monitor_and_respawn():
    rospy.init_node('key_respawner_simple', anonymous=True)

    global start_pose
    start_pose = Pose()
    start_pose.position.x = 0.0  
    start_pose.position.y = 0.0  
    start_pose.position.z = 0.0  
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 0.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 1.0

    # Start the key listener thread
    key_listener_thread = threading.Thread(target=listen_for_keys)
    key_listener_thread.daemon = True
    key_listener_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        monitor_and_respawn()
    except rospy.ROSInterruptException:
        pass
