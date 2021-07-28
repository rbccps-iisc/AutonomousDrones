#!/usr/bin/env python

# Code to control gcs station for multiple drones
# this code collects state information from each drone in the system, and coordinates their actions
# Usage - python gcs_control.py --ndr=#no_of_drones_per_group --nar=no_of_arucos_per_group --groups=#no_of_groups
#Example python gcs_control.py --ndr=2 --nar=2 --groups=1


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

		self.ndr = ndr		#number of drones in one group
		self.nar = nar		#number of aruco markers in one group
		self.groups = groups	#number of groups. Each group must have same no. of drones/arucos and setup of all groups must be symmetric with only difference being in their positions

		self.drone = [None] * (self.ndr*self.groups)
		self.aruco = [None] * (self.nar*self.groups)

		self.__active = [None] * (self.groups) 		#List of drone IDs which will be currently flying and carrying out mission
		self.__replacement = [None] * (self.groups)	#List of drone IDs which will be used to replace active drones in case their battery level falls below a certain percentage
		
		self.__pub_cmd = [None] * (self.ndr*self.groups)	#Publisher object for commands to be published to each drone

		#List of local frame GPS position difference between a particular drone and the first drone of the group
		self.__x_diff = [None]* (self.ndr*self.groups)	
		self.__y_diff = [None]* (self.ndr*self.groups)
		
		#Safe Home Position GPS coordinates. In this system, there is 1 safe location for each group (Can be extended to more later)
		self.home_lat = [float('nan')] * self.groups
		self.home_lon = [float('nan')] * self.groups
		self.home_alt = [float('nan')] * self.groups


	class drone_params():
		def __init__(self):
			#These parameters are taken from the drone's autopilot
			self._cur_x = 0			#Current x location of drone in local GPS coordinates
			self._cur_y = 0			#Current y location of drone in local GPS coordinates
			self._cur_alt = 0		#Current altitude of drone in local GPS coordinates
			self._roll = 0			#Current roll angle of drone in local GPS coordinates
			self._pitch = 0			#Current pitch angle of drone in local GPS coordinates
			self._yaw = 0			#Current yaw angle of drone in local GPS coordinates
			self._armed = None		#armed status of the drone
			self._waypoint = 1		#Current waypoint which the drone is at (Default 1st waypoint)
			self._status = 'unavailable'	#Message from drone as per its status. Need Replacement, Ready to takeoff, etc.
			self._bat = 100			#Battery level of drone 

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
			#GPS data from aruco
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
			
			self.drone[i] = self.drone_params()	#drone class objects

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
			
			#We can add further callbacks as the landing pad develops. Plans to add wind sensors, temp sensors, etc to the landing pad
			rospy.Subscriber(aruco_ID + '/mavros/global_position/global', NavSatFix, self.aruco[i]._global_pos_cb)
			rospy.Subscriber(aruco_ID + '/mavros/altitude', Altitude, self.aruco[i]._alt_cb)

		
		

	def set_home_positions(self):

		#Home position is a safe location where the drone can land for maintanence or in case of failure.
		#Each group of drones has one safe location (Can be extended later)
		
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
			
			#Finding the local coordinate differences between all drones in a particular group wrt 1st drone of the group
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
			
			#For initial trial. Here we just ask the 1st drone of the group to takeoff and start mission
			if not start:
		
				start = True

				cmd.command = active_ID + ' takeoff'	#1st drone takeoff command given
				#These parameters are default for the 1st sorty. relaying float('nan') causes drone to assume default values
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
				
				#When drone has reached the initial location for 1st sorty, GCS asks it to start mission.
				cmd.command = active_ID + ' do mission'
				self.__pub_cmd[self.__active[group_no]].publish(cmd)
				time.sleep(1)
				cmd.command = 'None'
				self.__pub_cmd[self.__active[group_no]].publish(cmd)
			
			#For subsequent sorties after 1st sorty
			else:
				#Condition when the active drone has low battery and seeks GCS for a replacement
				if self.drone[self.__active[group_no]]._status == active_ID + ' need replacement':
					
					#aruco GPS location where the active drone must go in order to land safely using MPC algorithm
					cmd.aruco_lat = self.aruco[self.__replacement[group_no]]._aruco_lat
					cmd.aruco_lon = self.aruco[self.__replacement[group_no]]._aruco_lon
					cmd.aruco_alt = self.aruco[self.__replacement[group_no]]._aruco_alt

					if  not self.drone[self.__replacement[group_no]]._armed:		#just a safety check to make sure replacement drone is not armed
	
						#Taking current mission info from active drone
						mission_height = self.drone[self.__active[group_no]]._cur_alt
						mission_waypoint = self.drone[self.__active[group_no]]._waypoint
						mission_x = self.drone[self.__active[group_no]]._cur_x
						mission_y = self.drone[self.__active[group_no]]._cur_y
						mission_yaw = self.drone[self.__active[group_no]]._yaw
						
						#Telling active drone that replacement drone is available and ready.
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
						
						#On getting the above information, the active drone increases altitude. GCS waits till this has finished
						while self.drone[self.__active[group_no]]._status != active_ID + ' ready for replacement':		
							continue
						
						#Commanding replacement drone to takeoff and position itself according to the current mission info
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
						
						#GCS waiting for replacement drone to assume position according to current mission info.
						while(self.drone[self.__replacement[group_no]]._status != replacement_ID + ' at location'):
							continue
						
						#Commanding active drone to start land sequence. This includes going to aruco location and landing accurately on landing pad using MPC algo.
						cmd.command = active_ID + ' land'
						cmd.x_diff = self.__x_diff[self.__active[group_no]]
						cmd.y_diff = self.__y_diff[self.__active[group_no]]
						self.__pub_cmd[self.__active[group_no]].publish(cmd)
						time.sleep(1)
						cmd.command = 'None'
						self.__pub_cmd[self.__active[group_no]].publish(cmd)
						
						#Commanding replacement drone (which has just now become the active drone) to continue mission
						if self.drone[self.__replacement[group_no]]._armed:
							cmd.command = replacement_ID + ' do mission'
							cmd.waypoint = mission_waypoint
							self.__pub_cmd[self.__replacement[group_no]].publish(cmd)
							time.sleep(1)
							cmd.command = 'None'
							self.__pub_cmd[self.__replacement[group_no]].publish(cmd)

						self.__active[group_no] = self.__replacement[group_no]
					
					#If replacement drone is armed then there is some error in the system
					#This throws an error message in such situation.
					#Failsafe procedure is not implemented yet, but can be found in the failsafe branch on github
					else:
						cmd.command = 'replacement for ' + active_ID + ' not ready'
						for i in range(init_drone_no, init_drone_no+self.ndr):
							self.__pub_cmd[i].publish(cmd)
						print('ERROR')
						exit()

			rate.sleep()

			
	#Function to find replacement drone for a particular group
	def __replacement_drone_search(self, init_drone_no, group_no):

		while not rospy.is_shutdown():
			
			#This loop checks battery percentage of all non active drones in the group, identifies the one with highest battery percentage and selects that as the replacement drone for this group
			#Loop runs continuously in parallel to the main operation. 
			for i in range(init_drone_no,init_drone_no+self.ndr):
				if (self.drone[i] != self.drone[self.__active[group_no]] and self.drone[i]._bat > self.drone[self.__replacement[group_no]]._bat):
					self.__replacement[group_no] = i


	def run_gcs(self):

		rospy.init_node('gcs_control')
		
		#Getting data for drones, arucos, and home locations
		self.init_gcs_for_drones()
		time.sleep(1)
		self.init_gcs_for_arucos()
		time.sleep(1)
		self.set_home_positions()
		time.sleep(1)
		
		threads = []
		
		#foreach group we initialize 2 threads - 1 to give and take commands from GCS to drones and 1 to select the current replacement drone for the group
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
