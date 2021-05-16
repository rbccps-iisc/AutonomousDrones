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


class GCScontrol():

	def __init__(self, ndr):

		self.ndr = ndr

		self.__home_lat = float('nan')
		self.__home_lon = float('nan')
		self.__home_alt = float('nan')
		#self.__call_home_pos = None
		#self.__call_home_alt = None
		#self.__drone_params[] = self.drone_params()

	'''
	def __global_pos_cb(self, data):
		self.__home_lat = data.latitude
		self.__home_lon = data.longitude
		#self.__call_home_pos.unregister()

	def __global_alt_cb(self, data):
		self.__home_alt = data.amsl
		#self.__call_home_alt.unregister()	
	'''

	class drone_params():

		def __init__(self):
			self._cur_x = 0
			self._cur_y = 0
			self._cur_alt = 0
			self._roll = 0
			self._pitch = 0
			self._yaw = 0
			self._armed = False
			self._waypoint = 1
			self._status = 'unavailable'
			self._bat = 100

		def _local_pos_cb(self, data):
			self._cur_x = data.pose.pose.position.x
			self._cur_y = data.pose.pose.position.y
			self._cur_alt = data.pose.pose.position.z

		def _ori_cb(self, data):
			orientation_q = data.orientation
			orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
			(self._roll, self._pitch, self._yaw) = euler_from_quaternion(orientation_list)

		def _armed_cb(self, data):
			self._armed = data.armed

		def _waypoint_cb(self, data):
			self._waypoint = data.current_seq

		def _status_cb(self, data):
			self._status = data.data

		def _bat_cb(data):
			self._bat = data.percentage

		def _bat_cb_sim(self, data):
			self._bat = data.data


	def control(self):

		rospy.init_node('gcs_control')
		rate = rospy.Rate(10)

		drone = [None]*self.ndr

		for i in range(self.ndr):
			drone[i] = self.drone_params()

		#call_home_pos = rospy.Subscriber('/drone1/mavros/global_position/global', NavSatFix, self.__global_pos_cb)
		#call_home_alt = rospy.Subscriber('/drone1/mavros/altitude',Altitude, self.__global_alt_cb)

		home_pos = rospy.wait_for_message('/drone1/mavros/global_position/global', NavSatFix)
		home_alt = rospy.wait_for_message('/drone1/mavros/altitude',Altitude)

		self.__home_lat = home_pos.latitude
		self.__home_lon = home_pos.longitude
		self.__home_alt = home_alt.amsl

		rospy.Subscriber('/drone1/mavros/global_position/local', Odometry, drone[0]._local_pos_cb)
		rospy.Subscriber('/drone1/mavros/imu/data', Imu, drone[0]._ori_cb)
		rospy.Subscriber('/drone1/mavros/state', State, drone[0]._armed_cb)
		rospy.Subscriber('/drone1/mavros/mission/waypoints', WaypointList, drone[0]._waypoint_cb)
		rospy.Subscriber('/drone1_status', String, drone[0]._status_cb)
		rospy.Subscriber('/drone1_battery', Int16, drone[0]._bat_cb_sim)
		pub_home_1 = rospy.Publisher('/drone1/mavros/global_position/home', HomePosition, queue_size=10)

		rospy.Subscriber('/drone2/mavros/global_position/local',Odometry, drone[1]._local_pos_cb)
		rospy.Subscriber('/drone2/mavros/imu/data', Imu, drone[1]._ori_cb)
		rospy.Subscriber('/drone2/mavros/state', State, drone[1]._armed_cb)
		rospy.Subscriber('/drone2/mavros/mission/waypoints', WaypointList, drone[1]._waypoint_cb)
		rospy.Subscriber('/drone2_status', String, drone[1]._status_cb)
		rospy.Subscriber('/drone2_battery', Int16, drone[1]._bat_cb_sim)
		pub_home_2 = rospy.Publisher('/drone2/mavros/global_position/home', HomePosition, queue_size=10)


		time.sleep(1)


		pub_home = HomePosition()
		pub_home.geo.latitude = self.__home_lat
		pub_home.geo.longitude = self.__home_lon
		pub_home.geo.altitude = self.__home_alt
		time.sleep(1)
		pub_home_1.publish(pub_home)
		time.sleep(1)
		pub_home_2.publish(pub_home)
		time.sleep(1)

		x_diff, y_diff = [None]*self.ndr , [None]*self.ndr

		x_diff[0] = drone[0]._cur_x - drone[0]._cur_x
		y_diff[0] = drone[0]._cur_y - drone[0]._cur_y

		x_diff[1] = drone[0]._cur_x - drone[1]._cur_x
		y_diff[1] = drone[0]._cur_y - drone[1]._cur_y


		#Checking if home location is set properly
		for i in range(self.ndr):
			if(drone[i]._cur_x+x_diff[i]>3 and drone[i]._cur_y+y_diff[i]>3):
				print('drone' + str(i+1) + ' Home Location ERROR')
				exit()


		pub_cmd = rospy.Publisher('gcs_command', gcs_msg, queue_size=1)
		cmd = gcs_msg()

		cmd.home_lat = self.__home_lat
		cmd.home_lon = self.__home_lon
		cmd.home_alt = self.__home_alt

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

				while(drone[0]._status!='drone1 at location'):
					continue

				cmd.command = 'drone1 do mission'
				pub_cmd.publish(cmd)
				time.sleep(1)

			else:

				if drone[0]._status == 'drone1 need replacement':

					if  not drone[1]._armed:

						mission_height = drone[0]._cur_alt
						mission_waypoint = drone[0]._waypoint
						mission_x = drone[0]._cur_x
						mission_y = drone[0]._cur_y
						mission_yaw = drone[0]._yaw

						cmd.command = 'drone1 replacement ready'
						cmd.x = drone[0]._cur_x
						cmd.y = drone[0]._cur_y
						cmd.x_diff = x_diff[0]
						cmd.y_diff = y_diff[0]
						cmd.height =  1.5 * drone[0]._cur_alt
						cmd.yaw = drone[0]._yaw
						pub_cmd.publish(cmd)
						time.sleep(1)

						while drone[0]._status != 'drone1 ready for replacement':		
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

						while(drone[1]._status!='drone2 at location'):
							continue

						cmd.command = 'drone1 land'
						cmd.x_diff = x_diff[0]
						cmd.y_diff = y_diff[0]
						pub_cmd.publish(cmd)
						time.sleep(1)
				
						if drone[1]._armed:
							cmd.command = 'drone2 do mission'
							cmd.waypoint = mission_waypoint
							pub_cmd.publish(cmd)
							time.sleep(1)
				
					else:
						cmd.command = 'replacement for drone1 not ready'
						pub_cmd.publish(cmd)
						print('ERROR')
						exit()

				if drone[1]._status == 'drone2 need replacement':

					if  not drone[0]._armed:

						mission_height = drone[1]._cur_alt
						mission_waypoint = drone[1]._waypoint
						mission_x = drone[1]._cur_x
						mission_y = drone[1]._cur_y
						mission_yaw = drone[1]._yaw

						cmd.command = 'drone2 replacement ready'
						cmd.x = drone[1]._cur_x 
						cmd.y = drone[1]._cur_y
						cmd.x_diff = x_diff[1]
						cmd.y_diff = y_diff[1]
						cmd.height =  1.5 * drone[1]._cur_alt
						cmd.yaw = drone[1]._yaw
						pub_cmd.publish(cmd)
						time.sleep(1)

						while drone[1]._status != 'drone2 ready for replacement':
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

						while(drone[0]._status!='drone1 at location'):
							continue

						cmd.command = 'drone2 land'
						cmd.x_diff = x_diff[1]
						cmd.y_diff = y_diff[1]
						pub_cmd.publish(cmd)
						time.sleep(1)
				
						if drone[0]._armed:
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
	
	parser = argparse.ArgumentParser(description='gcs_params')
	parser.add_argument('--ndr', default=2, type=int)

	args = parser.parse_args()

	#Making sure that drone number is valid
	if args.ndr<=1:
		print('number of drones invalid')
		exit()

	ndr = args.ndr

	gcs = GCScontrol(ndr)
	gcs.control()