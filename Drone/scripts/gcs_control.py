#!/usr/bin/env python

# Code to control gcs station for multiple drones
# this code collects state information from each drone in the system, and coordinates their actions
# Usage - python gcs_control.py


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import rospy, mavros, time
from mavros import command

import argparse
import threading

import numpy as np
from tf.transformations import euler_from_quaternion

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, TwistStamped, PoseStamped, PoseWithCovarianceStamped 

from std_msgs.msg import String, Int16
from gazebo_msgs.msg import ModelStates
from multidrone_mission.msg import gcs_msg

#number of drones/drones
ndr = 2

class drone_params(object):

	def __init__(self):
		self.cur_lat = 0
		self.cur_lon = 0
		self.cur_absalt = 0
		self.cur_x = 0
		self.cur_y = 0
		self.cur_alt = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.armed = False
		self.waypoint = 1
		self.status = 'unavailable'
		self.bat = 100

	def global_pos_cb(self, data):
		self.cur_lat = data.latitude
		self.cur_lon = data.longitude
		self.cur_absalt = data.altitude

	def local_pos_cb(self,data):
		self.cur_x = data.pose.pose.position.x
		self.cur_y = data.pose.pose.position.y
		self.cur_alt = data.pose.pose.position.z

	def ori_cb(self, data):
		orientation_q = data.orientation
		orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
		#Angles are in degrees
		#(self.roll, self.pitch, self.yaw) = (self.roll*180.0/3.1416, self.pitch*180.0/3.1416, self.yaw *180.0/3.1416)

	def armed_cb(self, data):
		self.armed = data.armed

	def waypoint_cb(self, data):
		self.waypoint = data.current_seq

	def status_cb(self, data):
		self.status = data.data

	def bat_cb(data):
		self.bat = data.percentage

	def bat_cb_sim(self, data):
		self.bat = data.data


def main():
	rospy.init_node('gcs_control')
	rate = rospy.Rate(10)
	
	drone = [None]*ndr

	for i in range(ndr):
		drone[i] = drone_params()

	rospy.Subscriber('/drone1/mavros/global_position/global', NavSatFix, drone[0].global_pos_cb)
	rospy.Subscriber('/drone1/mavros/global_position/local', Odometry, drone[0].local_pos_cb)
	rospy.Subscriber('/drone1/mavros/imu/data', Imu, drone[0].ori_cb)
	rospy.Subscriber('/drone1/mavros/state', State, drone[0].armed_cb)
	rospy.Subscriber('/drone1/mavros/mission/waypoints', WaypointList, drone[0].waypoint_cb)
	rospy.Subscriber('/drone1_status', String, drone[0].status_cb)
	rospy.Subscriber('/drone1_battery', Int16, drone[0].bat_cb_sim)
	pub_home_1 = rospy.Publisher('/drone1/mavros/global_position/home', HomePosition, queue_size=10)

	rospy.Subscriber('/drone2/mavros/global_position/global', NavSatFix, drone[1].global_pos_cb)
	rospy.Subscriber('/drone2/mavros/global_position/local',Odometry, drone[1].local_pos_cb)
	rospy.Subscriber('/drone2/mavros/imu/data', Imu, drone[1].ori_cb)
	rospy.Subscriber('/drone2/mavros/state', State, drone[1].armed_cb)
	rospy.Subscriber('/drone2/mavros/mission/waypoints', WaypointList, drone[1].waypoint_cb)
	rospy.Subscriber('/drone2_status', String, drone[1].status_cb)
	rospy.Subscriber('/drone2_battery', Int16, drone[1].bat_cb_sim)
	pub_home_2 = rospy.Publisher('/drone2/mavros/global_position/home', HomePosition, queue_size=10)

	
	time.sleep(1)
	pub_home = HomePosition()
	pub_home.geo.latitude = drone[0].cur_lat
	pub_home.geo.longitude = drone[0].cur_lon
	pub_home.geo.altitude = drone[0].cur_absalt
	time.sleep(1)
	pub_home_1.publish(pub_home)
	time.sleep(1)
	pub_home_2.publish(pub_home)
	time.sleep(1)

	x_diff, y_diff = [None]*ndr , [None]*ndr

	x_diff[0] = drone[0].cur_x - drone[0].cur_x
	y_diff[0] = drone[0].cur_y - drone[0].cur_y

	x_diff[1] = drone[0].cur_x - drone[1].cur_x
	y_diff[1] = drone[0].cur_y - drone[1].cur_y
	
	#Checking if home location is set properly
	for i in range(ndr):
		if(drone[i].cur_x+x_diff[i]>0.1 and drone[0].cur_y+y_diff[i]>0.1):
			print('drone' + str(i+1) + ' Home Location ERROR')
			exit()
		

	home_lat = drone[0].cur_lat
	home_lon = drone[0].cur_lon

	pub_cmd = rospy.Publisher('gcs_command', gcs_msg, queue_size=1)
	cmd = gcs_msg()

	cmd.home_lat = home_lat
	cmd.home_lon = home_lon


	start = False

	while not rospy.is_shutdown():	

		if not start:
	
			start = True
			
			time.sleep(1)
			cmd.command = 'drone1 takeoff'
			cmd.x_diff = float('nan')
			cmd.y_diff = float('nan')
			cmd.x = float('nan')
			cmd.y = float('nan')
			cmd.height = float('nan')
			cmd.yaw = float('nan')
			cmd.waypoint = 1
			pub_cmd.publish(cmd)
			time.sleep(1)

			while(drone[0].status!='drone1 at location'):
				continue

			cmd.command = 'drone1 do mission'
			pub_cmd.publish(cmd)
			time.sleep(1)

		else:

			if drone[0].status == 'drone1 need replacement':

				if  not drone[1].armed:

					mission_height = drone[0].cur_alt
					mission_waypoint = drone[0].waypoint
					mission_x = drone[0].cur_x
					mission_y = drone[0].cur_y
					mission_yaw = drone[0].yaw

					cmd.command = 'drone1 replacement ready'
					cmd.x = drone[0].cur_x
					cmd.y = drone[0].cur_y
					cmd.x_diff = x_diff[0]
					cmd.y_diff = y_diff[0]
					cmd.height =  1.5 * drone[0].cur_alt
					cmd.yaw = drone[0].yaw
					pub_cmd.publish(cmd)
					time.sleep(1)

					while drone[0].status != 'drone1 ready for replacement':		
						continue
					
					cmd.command = 'drone2 takeoff'
					cmd.x = mission_x
					cmd.y = mission_y
					cmd.x_diff = x_diff[1]
					cmd.y_diff = y_diff[1]
					cmd.height = mission_height
					cmd.yaw = mission_yaw
					pub_cmd.publish(cmd)
					time.sleep(1)

					while(drone[1].status!='drone2 at location'):
						continue

					cmd.command = 'drone1 land'
					cmd.x_diff = x_diff[0]
					cmd.y_diff = y_diff[0]
					pub_cmd.publish(cmd)
					time.sleep(1)
			
					if drone[1].armed:
						cmd.command = 'drone2 do mission'
						cmd.waypoint = mission_waypoint
						pub_cmd.publish(cmd)
						time.sleep(1)
			
				else:
					cmd.command = 'replacement for drone1 not ready'
					pub_cmd.publish(cmd)
					print('ERROR')
					exit()

			if drone[1].status == 'drone2 need replacement':

				if  not drone[0].armed:

					mission_height = drone[1].cur_alt
					mission_waypoint = drone[1].waypoint
					mission_x = drone[1].cur_x
					mission_y = drone[1].cur_y
					mission_yaw = drone[1].yaw

					cmd.command = 'drone2 replacement ready'
					cmd.x = drone[1].cur_x 
					cmd.y = drone[1].cur_y
					cmd.x_diff = x_diff[1]
					cmd.y_diff = y_diff[1]
					cmd.height =  1.5 * drone[1].cur_alt
					cmd.yaw = drone[1].yaw
					pub_cmd.publish(cmd)
					time.sleep(1)

					while drone[1].status != 'drone2 ready for replacement':
						continue
					
					cmd.command = 'drone1 takeoff'
					cmd.x = mission_x
					cmd.y = mission_y
					cmd.x_diff = x_diff[0]
					cmd.y_diff = y_diff[0]
					cmd.height = mission_height
					cmd.yaw = mission_yaw
					pub_cmd.publish(cmd)
					time.sleep(1)

					while(drone[0].status!='drone1 at location'):
						continue

					cmd.command = 'drone2 land'
					cmd.x_diff = x_diff[1]
					cmd.y_diff = y_diff[1]
					pub_cmd.publish(cmd)
					time.sleep(1)
			
					if drone[0].armed:
						cmd.command = 'drone1 do mission'
						cmd.waypoint = mission_waypoint
						pub_cmd.publish(cmd)
						time.sleep(1)
			
				else:
					cmd.command = 'replacement for drone2 not ready'
					pub_cmd.publish(cmd)
					print('ERROR')
					exit()

		rate.sleep()


if __name__ == '__main__':
	
	main()
