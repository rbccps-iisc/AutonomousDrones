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
import multiprocessing as mp

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

	def __init__(self, ndr, nar, groups):

		self.ndr = ndr
		self.nar = nar
		self.groups = groups

		self.drone = [None] * (self.ndr*self.groups)
		self.aruco = [None] * (self.nar*self.groups)

		self.__active = [None] * (self.groups) 			#List of drone IDs which will be currently flying and carrying out mission
		self.__replacement = [None] * (self.groups)		#List of drone IDs which will be used to replace active drones in case of battery level degradation
		
		self.__pub_cmd = [None] * (self.ndr*self.groups)

		self.__x_diff = [None]* (self.ndr*self.groups)
		self.__y_diff = [None]* (self.ndr*self.groups)

		self.home_lat = [float('nan')] * self.groups
		self.home_lon = [float('nan')] * self.groups
		self.home_alt = [float('nan')] * self.groups


	class drone_params():
		def __init__(self):
			self._cur_x = 0
			self._cur_y = 0
			self._cur_alt = 0
			self._roll = 0
			self._pitch = 0
			self._yaw = 0
			self._armed = None
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


	class aruco_params():
		def __init__(self):
			self._aruco_lat = float('nan')
			self._aruco_lon = float('nan')
			self._aruco_alt = float('nan')

		def _global_pos_cb(self, data):
			self._aruco_lat = data.latitude
			self._aruco_lon = data.longitude

		def _alt_cb(self, data):
			self._aruco_alt = data.amsl


	#Getting parameters for all drones in system
	def init_gcs_for_drones(self):

		tot_drones = self.ndr*self.groups

		for i in range(tot_drones):
			
			self.drone[i] = self.drone_params()

			drone_ID = 'drone' + str(i+1)

			rospy.Subscriber(drone_ID + '/mavros/global_position/local', Odometry, self.drone[i]._local_pos_cb)
			rospy.Subscriber(drone_ID + '/mavros/imu/data', Imu, self.drone[i]._ori_cb)
			rospy.Subscriber(drone_ID + '/mavros/state', State, self.drone[i]._armed_cb)
			rospy.Subscriber(drone_ID + '/mavros/mission/waypoints', WaypointList, self.drone[i]._waypoint_cb)
			rospy.Subscriber(drone_ID + '_status', String, self.drone[i]._status_cb)
			rospy.Subscriber(drone_ID + '_battery', Int16, self.drone[i]._bat_cb_sim)


	#Getting parameters for all aruco markers in system
	def init_gcs_for_arucos(self):

		tot_arucos = self.nar*self.groups

		for i in range(tot_arucos):
			
			self.aruco[i] = self.aruco_params()

			#for testing purposes, we can only take drone ID's because aruco computers are not yet ready.
			#However since 1st drone of each group starts on top of aruco marker, this is also accurate as of now.
			#Later, we must replace this with actual configuration of arucos
			aruco_ID = 'drone' + str(i+1)

			rospy.Subscriber(aruco_ID + '/mavros/global_position/global', NavSatFix, self.aruco[i]._global_pos_cb)
			rospy.Subscriber(aruco_ID + '/mavros/altitude', Altitude, self.aruco[i]._alt_cb)

		
		

	def set_home_positions(self):

		#Home position is a safe location where the drone can land for maintanence or in case of failure.
		#It is also the local (0,0,0) coordinate for a particular group of drones
		
		#Setting the location of 1st aruco marker of each group as the home position for that group of drones
		#If all home locations have a gps, then we can set home location directly using that gps' coordinates
		for i in range(self.groups):
			self.home_lat[i] = self.aruco[i*self.nar]._aruco_lat
			self.home_lon[i] = self.aruco[i*self.nar]._aruco_lon
			self.home_alt[i] = self.aruco[i*self.nar]._aruco_alt



	#Coordinate between drones in a particular group
	def control(self, init_drone_no, group_no):

		rate = rospy.Rate(10)
		
		pub_home_loc = HomePosition()
		pub_home_loc.geo.latitude = self.home_lat[group_no]
		pub_home_loc.geo.longitude = self.home_lon[group_no]
		pub_home_loc.geo.altitude = self.home_alt[group_no]

		#Setting common home location for all drones in group
		for i in range(init_drone_no, init_drone_no+self.ndr):
			drone_ID = 'drone' + str(i+1)
			pub_home = rospy.Publisher(drone_ID + '/mavros/global_position/home', HomePosition, queue_size=1)
			time.sleep(1)
			pub_home.publish(pub_home_loc)
			time.sleep(1)
			
			#Nullifying the local coordinate differences between all drones in a particular group wrt 1st drone of the group
			self.__x_diff[i] = self.drone[init_drone_no]._cur_x - self.drone[i]._cur_x
			self.__y_diff[i] = self.drone[init_drone_no]._cur_y - self.drone[i]._cur_y
			time.sleep(1)
			print(drone_ID, self.__x_diff[i], self.__y_diff[i])	
		
			#Publisher to broadcast gcs command
			self.__pub_cmd[i] = rospy.Publisher(drone_ID+'_gcs_command', gcs_msg, queue_size=1)
			time.sleep(1)
		
		cmd = gcs_msg()
		time.sleep(1)

		cmd.home_lat = self.home_lat[group_no]
		cmd.home_lon = self.home_lon[group_no]
		cmd.home_alt = self.home_alt[group_no]

		time.sleep(1)

		start = False

		while not rospy.is_shutdown():

			active_ID = 'drone' + str(self.__active[group_no]+1)
			replacement_ID = 'drone' + str(self.__replacement[group_no]+1)

			if not start:
		
				start = True

				cmd.command = active_ID + ' takeoff'
				cmd.x_diff = float('nan')
				cmd.y_diff = float('nan')
				cmd.x = float('nan')
				cmd.y = float('nan')
				cmd.height = float('nan')
				cmd.yaw = float('nan')
				cmd.waypoint = 1
				self.__pub_cmd[self.__active[group_no]].publish(cmd)
				time.sleep(1)
				cmd.command = 'None'
				self.__pub_cmd[self.__active[group_no]].publish(cmd)
				

				while(self.drone[self.__active[group_no]]._status != active_ID + ' at location'):
					continue

				cmd.command = active_ID + ' do mission'
				self.__pub_cmd[self.__active[group_no]].publish(cmd)
				time.sleep(1)
				cmd.command = 'None'
				self.__pub_cmd[self.__active[group_no]].publish(cmd)

			else:

				if self.drone[self.__active[group_no]]._status == active_ID + ' need replacement':

					cmd.aruco_lat = self.aruco[self.__replacement[group_no]]._aruco_lat
					cmd.aruco_lon = self.aruco[self.__replacement[group_no]]._aruco_lon
					cmd.aruco_alt = self.aruco[self.__replacement[group_no]]._aruco_alt

					if  not self.drone[self.__replacement[group_no]]._armed:

						mission_height = self.drone[self.__active[group_no]]._cur_alt
						mission_waypoint = self.drone[self.__active[group_no]]._waypoint
						mission_x = self.drone[self.__active[group_no]]._cur_x
						mission_y = self.drone[self.__active[group_no]]._cur_y
						mission_yaw = self.drone[self.__active[group_no]]._yaw

						cmd.command = active_ID + ' replacement ready'
						cmd.x = self.drone[self.__active[group_no]]._cur_x
						cmd.y = self.drone[self.__active[group_no]]._cur_y
						cmd.x_diff = self.__x_diff[self.__active[group_no]]
						cmd.y_diff = self.__y_diff[self.__active[group_no]]
						cmd.height =  1.5 * self.drone[self.__active[group_no]]._cur_alt
						cmd.yaw = self.drone[self.__active[group_no]]._yaw
						self.__pub_cmd[self.__active[group_no]].publish(cmd)
						time.sleep(1)
						cmd.command = 'None'
						self.__pub_cmd[self.__active[group_no]].publish(cmd)

						while self.drone[self.__active[group_no]]._status != active_ID + ' ready for replacement':		
							continue
						
						cmd.command = replacement_ID + ' takeoff'
						cmd.x = mission_x
						cmd.y = mission_y
						cmd.x_diff = self.__x_diff[self.__replacement[group_no]]
						cmd.y_diff = self.__y_diff[self.__replacement[group_no]]
						cmd.height = mission_height
						cmd.yaw = mission_yaw
						self.__pub_cmd[self.__replacement[group_no]].publish(cmd)
						time.sleep(1)
						cmd.command = 'None'
						self.__pub_cmd[self.__replacement[group_no]].publish(cmd)

						while(self.drone[self.__replacement[group_no]]._status != replacement_ID + ' at location'):
							continue

						cmd.command = active_ID + ' land'
						cmd.x_diff = self.__x_diff[self.__active[group_no]]
						cmd.y_diff = self.__y_diff[self.__active[group_no]]
						self.__pub_cmd[self.__active[group_no]].publish(cmd)
						time.sleep(1)
						cmd.command = 'None'
						self.__pub_cmd[self.__active[group_no]].publish(cmd)
				
						if self.drone[self.__replacement[group_no]]._armed:
							cmd.command = replacement_ID + ' do mission'
							cmd.waypoint = mission_waypoint
							self.__pub_cmd[self.__replacement[group_no]].publish(cmd)
							time.sleep(1)
							cmd.command = 'None'
							self.__pub_cmd[self.__replacement[group_no]].publish(cmd)

						self.__active[group_no] = self.__replacement[group_no]
				
					else:
						cmd.command = 'replacement for ' + active_ID + ' not ready'
						for i in range(init_drone_no, init_drone_no+self.ndr):
							self.__pub_cmd[i].publish(cmd)
						print('ERROR')
						exit()

			rate.sleep()


	def __replacement_drone_search(self, init_drone_no, group_no):

		while not rospy.is_shutdown():
			for i in range(init_drone_no,init_drone_no+self.ndr):
				if (self.drone[i] != self.drone[self.__active[group_no]] and self.drone[i]._bat > self.drone[self.__replacement[group_no]]._bat):
					self.__replacement[group_no] = i


	def run_gcs(self):

		rospy.init_node('gcs_control')

		self.init_gcs_for_drones()
		time.sleep(1)
		self.init_gcs_for_arucos()
		time.sleep(1)
		self.set_home_positions()
		time.sleep(1)
		
		threads = []
		
		for i in range(self.groups):

			init_drone_no = i*self.ndr
			group_no = i
			
			self.__active[group_no] = init_drone_no
			self.__replacement[group_no] = init_drone_no+1

			t1 = threading.Thread(target=self.control, args=(init_drone_no, group_no))
			threads.append(t1)
			t2 = threading.Thread(target=self.__replacement_drone_search, args=(init_drone_no, group_no))
			threads.append(t2)
		
		#Start all threads
		[x.start() for x in threads]
		

if __name__ == '__main__':
	
	parser = argparse.ArgumentParser(description='gcs_params')
	parser.add_argument('--ndr', default=3, type=int)			#no. of drones in one group
	parser.add_argument('--nar', default=3, type=int)			#no. of aruco markers in one group
	parser.add_argument('--groups', default=1, type=int)		#no. of groups
	
	args = parser.parse_args()

	#Making sure that drone number is valid
	if args.ndr<=1:
		print('number of drones invalid')
		exit()

	ndr = args.ndr
	nar = args.nar
	groups = args.groups
	
	gcs = GCScontrol(ndr, nar, groups)
	gcs.run_gcs()