from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import rospy, mavros, time
from mavros import command

from opencv.lib_aruco_pose import *
from tf.transformations import euler_from_quaternion

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, TwistStamped, PoseStamped

#number of drones
ndr = 3

class Mission(object):

    def __init__(self, vel_pub):
        self.cur_x = 0
        self.cur_y = 0
        self.cur_z = 0  
        self.roll = 0
        self.pitch = 0
        self.yaw = 0 

    def pos_cb(self, data):
        self.cur_x = data.pose.position.x
        self.cur_y = data.pose.position.y
        self.cur_z = data.pose.position.z

    def rotation_cb(self, data):
        orientation_q = data.orientation
        orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        #Angles are in degrees
        (self.roll, self.pitch, self.yaw) = (self.roll*180.0/3.1416, self.pitch*180.0/3.1416, self.yaw *180.0/3.1416)

def main():
    rospy.init_node('mavros_control_drone2')
    rate = rospy.Rate(10)
    mavros.set_namespace('drone2/mavros')

    #register publisher
    vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size = 3) 
    
    drone = Mission(vel_pub)

    #register subscribers
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, drone.pos_cb)
    rospy.Subscriber(mavros.get_topic('imu', 'data'), Imu, drone.rotation_cb)


    # setup services for arming, takeoff, etc
    set_arming = rospy.ServiceProxy('/drone2/mavros/cmd/arming', CommandBool)
    set_mode = rospy.ServiceProxy('/drone2/mavros/set_mode', SetMode)
    set_takeoff = rospy.ServiceProxy('/drone2/mavros/cmd/takeoff', CommandTOL)
    set_landing = rospy.ServiceProxy('/drone2/mavros/cmd/land', CommandTOL)

    setpoint_msg = TwistStamped(header=Header(stamp=rospy.Time.now()))

    #Change mode to guided to accept commands
    set_mode(custom_mode='GUIDED')

    #Arm the drone
    mavros.command.arming(True)
    time.sleep(1)

       
    set_takeoff(0, 0, None, None, 5)


    while not rospy.is_shutdown(): 
        rate.sleep()
       

if __name__ == '__main__':
    main()
