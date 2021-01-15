#!/usr/bin/env python3

# Code to control drone1
# running this code should cause all drone1 to takeoff simultaneously and hover at a height of 5m
# Usage - python3 drone1_mission.py


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import rospy, mavros, time
from mavros import command

import numpy as np
from tf.transformations import euler_from_quaternion

from mavros_msgs.msg import *
from mavros_msgs.srv import *

from std_msgs.msg import Header
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, TwistStamped, PoseStamped, PoseWithCovarianceStamped 

from std_msgs.msg import String, Int16
from gazebo_msgs.msg import ModelStates
from multidrone_mission.msg import drone

#number of drones/robots
ndr = 2

class drone_params(object):

	def __init__(self):
		self.cur_lat = 0
		self.cur_lon = 0
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

	def local_pos_cb(self,data):
		self.cur_x = data.pose.position.x
		self.cur_y = data.pose.position.y
		self.cur_alt = data.pose.position.z

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

	def bat_cb(self, data):
		self.bat = data.data


def main():
	rospy.init_node('gcs_control')
	rate = rospy.Rate(10)
	
	robot = [None]*ndr

	for i in range(ndr):
		robot[i] = drone_params()

	rospy.Subscriber('/drone1/mavros/global_position/global', NavSatFix, robot[0].global_pos_cb)
	rospy.Subscriber('/drone1/mavros/local_position/pose', PoseStamped, robot[0].local_pos_cb)
	rospy.Subscriber('/drone1/mavros/imu/data', Imu, robot[0].ori_cb)
	rospy.Subscriber('/drone1/mavros/state', State, robot[0].armed_cb)
	rospy.Subscriber('/drone1/mavros/mission/waypoints', WaypointList, robot[0].waypoint_cb)
	rospy.Subscriber('/drone1_status', String, robot[0].status_cb)
	rospy.Subscriber('/drone1_battery', Int16, robot[0].bat_cb)

	rospy.Subscriber('/drone2/mavros/global_position/global', NavSatFix, robot[1].global_pos_cb)
	rospy.Subscriber('/drone2/mavros/local_position/pose', PoseStamped, robot[1].local_pos_cb)
	rospy.Subscriber('/drone2/mavros/imu/data', Imu, robot[1].ori_cb)
	rospy.Subscriber('/drone2/mavros/state', State, robot[1].armed_cb)
	rospy.Subscriber('/drone2/mavros/mission/waypoints', WaypointList, robot[1].waypoint_cb)
	rospy.Subscriber('/drone2_status', String, robot[1].status_cb)
	rospy.Subscriber('/drone2_battery', Int16, robot[1].bat_cb)

	#rospy.Subscriber('/drone3/mavros/global_position/global', NavSatFix, robot[2].global_pos_cb)
	#rospy.Subscriber('/drone3/mavros/local_position/pose', PoseStamped, robot[2].local_pos_cb)
	#rospy.Subscriber('/drone3/mavros/imu/data', Imu, robot[2].ori_cb)
	#rospy.Subscriber('/drone3/mavros/state', State, robot[2].armed_cb)
	#rospy.Subscriber('/drone3/mavros/mission/waypoints', WaypointList, waypoint_cb)
	#rospy.Subscriber('/drone3_status', String, robot[2].status_cb)
	#rospy.Subscriber('/drone3_battery', Int16, robot[2].bat_cb)

	pub_cmd = rospy.Publisher('gcs_command', drone, queue_size=1)
	cmd = drone()
	
	start = False


	while not rospy.is_shutdown():

		if not start:

			start = True
			
			time.sleep(1)
			cmd.command = 'drone1 takeoff'
			cmd.lat = float('nan')
			cmd.lon = float('nan')
			cmd.height = float('nan')
			cmd.yaw = float('nan')
			cmd.waypoint = 1
			pub_cmd.publish(cmd)
			time.sleep(1)

			cmd.command = 'none'
			pub_cmd.publish(cmd)


			time.sleep(50)
			if robot[0].armed:
				cmd.command = 'drone1 do mission'
				pub_cmd.publish(cmd)
				time.sleep(1)
			else:
				print('ERROR')
				exit()

		else:

			if robot[0].status == 'drone1 need replacement':

				if  not robot[1].armed:
				
					cmd.command = 'drone1 replacement ready'
					cmd.lat = robot[0].cur_lat
					cmd.lon = robot[0].cur_lon
					cmd.height =  1.5 * robot[0].cur_alt
					cmd.yaw = robot[0].yaw
					mission_height = robot[0].cur_alt
					pub_cmd.publish(cmd)
					time.sleep(1)

					while robot[0].status != 'drone1 ready for replacement':
						cmd.command = 'none'
						pub_cmd.publish(cmd)
						time.sleep(1)
						continue
					
					cmd.command = 'drone2 takeoff'
					cmd.lat = robot[0].cur_lat
					cmd.lon = robot[0].cur_lon
					cmd.height = mission_height
					cmd.yaw = robot[0].yaw
					cmd.waypoint = robot[0].waypoint
					pub_cmd.publish(cmd)
					time.sleep(1)

					while abs(robot[1].cur_x - robot[0].cur_x) >= 0.3 and abs(robot[1].cur_y - robot[0].cur_y) >= 0.3 :
						print(abs(robot[1].cur_x - robot[0].cur_x))
						print(abs(robot[1].cur_y - robot[0].cur_y))
						cmd.command = 'none'
						pub_cmd.publish(cmd)
						time.sleep(1)
						continue

					cmd.command = 'drone1 land'
					pub_cmd.publish(cmd)
					time.sleep(1)
			
					if robot[1].armed:
						cmd.command = 'drone2 do mission'
						pub_cmd.publish(cmd)
						time.sleep(1)
			
				else:
					cmd.command = 'replacement for drone1 not ready'
					pub_cmd.publish(cmd)
					print('ERROR')
					exit()

			if robot[1].status == 'drone2 need replacement':

				if  not robot[0].armed:
				
					cmd.command = 'drone2 replacement ready'
					cmd.lat = robot[1].cur_lat
					cmd.lon = robot[1].cur_lon
					cmd.height =  1.5 * robot[1].cur_alt
					mission_height = robot[1].cur_alt
					pub_cmd.publish(cmd)
					time.sleep(1)

					while robot[1].status != 'drone2 ready for replacement':
						cmd.command = 'none'
						pub_cmd.publish(cmd)
						time.sleep(1)
						continue
					
					cmd.command = 'drone1 takeoff'
					cmd.lat = robot[1].cur_lat
					cmd.lon = robot[1].cur_lon
					cmd.height = mission_height
					cmd.yaw = robot[1].yaw
					cmd.waypoint = robot[1].waypoint
					pub_cmd.publish(cmd)
					time.sleep(1)

					while abs(robot[0].cur_x - robot[1].cur_x) >= 0.1 and abs(robot[0].cur_y - robot[1].cur_y) >= 0.1 :
						cmd.command = 'none'
						pub_cmd.publish(cmd)
						time.sleep(1)
						continue

					cmd.command = 'drone2 land'
					pub_cmd.publish(cmd)
					time.sleep(1)
			
					if robot[1].armed:
						cmd.command = 'drone1 do mission'
						pub_cmd.publish(cmd)
						time.sleep(1)
			
				else:
					cmd.command = 'replacement for drone2 not ready'
					pub_cmd.publish(cmd)
					print('ERROR')
					exit()


		cmd.command = 'none'
		pub_cmd.publish(cmd)
		rate.sleep()


if __name__ == '__main__':
	main()
