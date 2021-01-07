#!/usr/bin/env python3

# Code to control drone1
# running this code should cause all drone1 to takeoff simultaneously and hover at a height of 5m
# Usage - python3 drone1_mission.py


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import rospy, mavros, time
from mavros import command

from tf.transformations import euler_from_quaternion
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet
from mavros_msgs.msg import  ParamValue
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PointStamped, TwistStamped, PoseStamped

from std_msgs.msg import String, Int16
from gazebo_msgs.msg import ModelStates
from multidrone_mission.msg import drone

#number of drones
ndr = 3

class drone_params(object):

	def __init__(self):
		self.cur_x = 0
		self.cur_y = 0
		self.cur_z = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.status = 'unavailable'
		self.bat = 100

	def pos_cb(self, data):
		self.cur_x = data.latitude
		self.cur_y = data.longitude
		self.cur_z = data.altitude

	def ori_cb(self, data):
		orientation_q = data.orientation
		orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
		#Angles are in degrees
		(self.roll, self.pitch, self.yaw) = (self.roll*180.0/3.1416, self.pitch*180.0/3.1416, self.yaw *180.0/3.1416)

	def status_cb(self, data):
		self.status = data.data

	def bat_cb(self, data):
		self.bat = data.data


def main():
	rospy.init_node('gcs_control')
	rate = rospy.Rate(10)
	
	drone = [None]*ndr

	for i in range(ndr):
		drone[i] = drone_params()

	rospy.Subscriber('/drone1/mavros/global_position/global', NavSatFix, drone[0].pos_cb)
	rospy.Subscriber('/drone1/mavros/imu/data', Imu, drone[0].ori_cb)
	#rospy.Subscriber('/drone1_status', String, drone[0].status_cb)
	#rospy.Subscriber('/drone1_battery', Int16, drone[0].bat_cb)

	rospy.Subscriber('/drone2/mavros/global_position/global', NavSatFix, drone[1].pos_cb)
	rospy.Subscriber('/drone2/mavros/imu/data', Imu, drone[1].ori_cb)
	#rospy.Subscriber('/drone2_status', String, drone[1].status_cb)
	#rospy.Subscriber('/drone2_battery', Int16, drone[1].bat_cb)

	rospy.Subscriber('/drone3/mavros/global_position/global', NavSatFix, drone[2].pos_cb)
	rospy.Subscriber('/drone3/mavros/imu/data', Imu, drone[2].ori_cb)
	#rospy.Subscriber('/drone3_status', String, drone[2].status_cb)
	#rospy.Subscriber('/drone3_battery', Int16, drone[2].bat_cb)

	pub_cmd = rospy.Publisher('gcs_command', String, queue_size=10)
	
	start = False

	while not rospy.is_shutdown(): 

		
		print('drone 1 = %f' %(drone[0].cur_z))
		#print('drone 2 = %f' %(drone[1].cur_z))
		#print('drone 3 = %f' %(drone[2].cur_z))

		if not start:
			time.sleep(0.2)
			pub_cmd.publish('drone1_takeoff')
			time.sleep(0.2)
			start = True
		
		else:
			if drone[0].bat<=50 and drone[0].status=='flying':  
				pub_cmd.publish('drone1_land')
				time.sleep(0.2)
				pub_cmd.publish('drone2_takeoff')
				time.sleep(0.2)

			if drone[1].bat<=50 and drone[1].status=='flying':  
				pub_cmd.publish('drone2_land')
				time.sleep(0.2)
				pub_cmd.publish('drone3_takeoff')
				time.sleep(0.2)

			if drone[2].bat<=50 and drone[2].status=='flying':  
				pub_cmd.publish('drone3_land')
				time.sleep(0.2)
				pub_cmd.publish('drone1_takeoff')
				time.sleep(0.2)

		pub_cmd.publish('none')

		rate.sleep()

if __name__ == '__main__':
	main()
