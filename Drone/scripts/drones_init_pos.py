#!/usr/bin/env python

# This program is a serves as a rosnode which initializes the pose of the multiple drones in an open gazebo_ros node 
# Usage - rosrun scripts random_location_aruco.py

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# function to set drone location
def location_drone_1(pub):
    new_pose = ModelState()
    new_pose.model_name = 'iris_demo_9002'
    new_pose.pose.position.x = -2
    new_pose.pose.position.y = 0
    new_pose.pose.position.z = 0
    [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w] = quaternion_from_euler(0,0,0)
    rospy.sleep(1)
    pub.publish(new_pose)
    return
def location_drone_2(pub):
    new_pose = ModelState()
    new_pose.model_name = 'iris_demo_9012'
    new_pose.pose.position.x = 0
    new_pose.pose.position.y = 0
    new_pose.pose.position.z = 0
    [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w] = quaternion_from_euler(0,0,0)
    rospy.sleep(1)
    pub.publish(new_pose)
    return
def location_drone_3(pub):
    new_pose = ModelState()
    new_pose.model_name = 'iris_demo_9022'
    new_pose.pose.position.x = 2
    new_pose.pose.position.y = 0
    new_pose.pose.position.z = 0
    [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w] = quaternion_from_euler(0,0,0)
    rospy.sleep(1)
    pub.publish(new_pose)
    return

if __name__=='__main__':
    rospy.init_node('pose_publisher')
    pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)  #Publisher Node
    location_drone_1(pub)
    location_drone_2(pub)
    location_drone_3(pub)
    rospy.sleep(5)
    print("DONE")
    
    
