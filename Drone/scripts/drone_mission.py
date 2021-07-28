#!/usr/bin/env python

# Code to control drone
# running this code should cause drone to take commands from GCS and act accordingly
# Usage - python drone_mission.py --dr_no=$drone_id_number


from os import sys, path

sys.path.append('../MPC_lander')
from drone_control import LandUsingMPC

#sys.path.append('../opencv')
#from lib_aruco_pose import ArucoSingleTracker

import argparse
import threading
import time, math
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Header

import rospy, mavros
from mavros import command

from std_msgs.msg import String, Int16, Bool

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry

from multidrone_mission.msg import gcs_msg


class DroneMission():

	def __init__(self, drone_ID, toh, WP, safe, low, critical, Simulation):

		#Public Class Variables
		self.drone_ID = drone_ID	#drone_ID = drone1, drone2, drone3...
		self.toh = toh			#takeoff height
		self.WP = WP			#list of predefined waypoints
		self.safe = safe		#min. battery level required for drone to takeoff
		self.low = low			#batery level at which drone stops doing mission and contacts GCS 
		self.critical = critical	#critical battery level (not used currently)
		self.Simulation = Simulation	#If true, then Simulation setup. For field trials make False.

		# battery percentage (for simulation only)
		self.__bat = 100
		# drone flight status
		self.__status = 'not yet initiated'
		# drone armed status
		self.__armed = False

		#To detect first time landing
		self.__first_land = True

		#Command from GCS telling this Drone what to do
		self.__gcs_cmd_str = 'none'			
		
		#Home (Safe) GPS location for this drone (Currently taken as initial location of 1st drone of this group)
		self.__gcs_cmd_home_lat = float('nan')		
		self.__gcs_cmd_home_lon = float('nan')		
		self.__gcs_cmd_home_alt = float('nan')
		
		#Aruco GPS location given by GCS to which this drone must go to in order to land safely
		self.__gcs_cmd_aruco_lat = float('nan')
		self.__gcs_cmd_aruco_lon = float('nan')
		self.__gcs_cmd_aruco_alt = float('nan')
		
		#Difference of this drone's local GPS coordinates wrt the local GPS coordinates of the 1st drone in this group
		self.__gcs_cmd_x_diff = float('nan')
		self.__gcs_cmd_y_diff = float('nan')
		
		#Desired GPS location from GCS in local coordinate system
		self.__gcs_cmd_x = float('nan')
		self.__gcs_cmd_y = float('nan')
		self.__gcs_cmd_height = float('nan')
		
		#Desired yaw position from GCS (in degrees)
		self.__gcs_cmd_yaw = float('nan')
		
		#Next waypoint in mission to which the drone must go to
		self.__gcs_cmd_wp = 1

		#This drone's local GPS coordinates
		self.__cur_x = 0
		self.__cur_y = 0
		self.__cur_alt = 0

		#Current drone's attitude
		self.__cur_roll = 0
		self.__cur_pitch = 0
		self.__cur_yaw = 0

		#List of waypoints fed to the drone, after converting from meters to Lat/Lon
		self.__dLat = []
		self.__dLon = []

		# Definitions for rospy services
		self.__set_mode = rospy.ServiceProxy(drone_ID+'/mavros/set_mode', SetMode)
		self.__set_cur_waypoint = rospy.ServiceProxy(drone_ID+'/mavros/mission/set_current', WaypointSetCurrent)
		self.__waypoints_clean = rospy.ServiceProxy(drone_ID+'/mavros/mission/clear', WaypointClear)
		self.__set_waypoint = rospy.ServiceProxy(drone_ID+'/mavros/mission/push', WaypointPush)
		self.__set_param = rospy.ServiceProxy(drone_ID+'/mavros/param/set',ParamSet)

		#Publisher function to publish status of drone
		self.__pub_status = rospy.Publisher(drone_ID+'_status', String, queue_size=1)
		#Publisher function to publish whether or not to run aruco detection code
		self.__pub_aruco = rospy.Publisher(drone_ID+'/run_aruco_detect', Bool, queue_size=1)


	def __armed_cb(self, data):
		self.__armed = data.armed

	def __bat_cb(self, data):
		self.__bat = data.percentage

	def __location_cb(self, data):
		self.__cur_x = data.pose.pose.position.x
		self.__cur_y = data.pose.pose.position.y
		self.__cur_alt = data.pose.pose.position.z

	def __ori_cb(self, data):
		orientation_q = data.orientation
		orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
		(self.__cur_roll, self.__cur_pitch, self.__cur_yaw) = euler_from_quaternion(orientation_list)

	def __gcs_command_cb(self, data):
		self.__gcs_cmd_str = data.command
		self.__gcs_cmd_home_lat = data.home_lat
		self.__gcs_cmd_home_lon = data.home_lon
		self.__gcs_cmd_home_alt = data.home_alt
		self.__gcs_cmd_aruco_lat = data.aruco_lat
		self.__gcs_cmd_aruco_lon = data.aruco_lon
		self.__gcs_cmd_aruco_alt = data.aruco_alt
		self.__gcs_cmd_x_diff = data.x_diff
		self.__gcs_cmd_y_diff = data.y_diff
		self.__gcs_cmd_x = data.x
		self.__gcs_cmd_y = data.y
		self.__gcs_cmd_height = data.height
		self.__gcs_cmd_yaw = data.yaw 	
		self.__gcs_cmd_wp = data.waypoint
	
	#Function to keep the mission continuous. When drone reaches last waypoint, next waypoint is updated to 1
	def __control_mission_cb(self, data):
		cur_wp = data.current_seq
		#W[last]==W[first] (Required to keep mission continued)
		if cur_wp==(len(self.WP)+1):
			self.__set_cur_waypoint(wp_seq=1)
			cur_wp = 1

	#Function to update the drone's waypoint list
	def __waypoint_dataset(self):

		W = []
		WP_number = len(self.WP)

		wp = Waypoint()
		wp.frame = 3							#FRAME_GLOBAL_REL_ALT
		wp.command = 22  						#Takeoff
		wp.is_current = False
		wp.autocontinue = True
		wp.param1 = 1 							#delay 
		wp.param2 = 1							#Accept Radius
		wp.param3 = 1							#Pass Radius
		wp.param4 = 0							#Yaw
		wp.x_lat = self.__gcs_cmd_home_lat		#Latitude
		wp.y_long = self.__gcs_cmd_home_lon		#Longitude
		wp.z_alt = self.toh						#altitude
		W.append(wp)

		for i in range(WP_number):	
			wp = Waypoint()
			wp.frame = 3												#FRAME_GLOBAL_REL_ALT
			wp.command = 16  											#Navigate to waypoint
			wp.is_current = False
			wp.autocontinue = True
			wp.param1 = 1 												#delay
			wp.param2 = 1												#Accept Radius
			wp.param3 = 0												#Pass Radius
			wp.param4 = self.WP[i][2]									#Yaw
			wp.x_lat = self.__gcs_cmd_home_lat + self.__dLat[i]			#Latitude
			wp.y_long = self.__gcs_cmd_home_lon + self.__dLon[i]		#Longitude
			wp.z_alt = self.toh											#altitude
			W.append(wp)

		wp = Waypoint()
		wp.frame = 3												#FRAME_GLOBAL_REL_ALT
		wp.command = 16  											#Navigate to waypoint.
		wp.is_current = False
		wp.autocontinue = True
		wp.param1 = 1  												#delay
		wp.param2 = 1												#Accept Radius
		wp.param3 = 0												#Pass Radius
		wp.param4 = self.WP[0][2]									#Yaw
		wp.x_lat = self.__gcs_cmd_home_lat + self.__dLat[0]			#Latitude
		wp.y_long = self.__gcs_cmd_home_lon + self.__dLon[0]		#Longitude
		wp.z_alt = self.toh											#altitude
		W.append(wp)
		
		return(W)


	#Function to convert local coordinates (x,y) to global coordinates (latitude,longitude)
	def __dis_to_gps(self, x_meters,y_meters):

		earth_radius=6378137.0 #Radius of "spherical" earth

		#Converting desired trajectories of drone in meters to GPS coordinates (in radians)
		dLat = x_meters/earth_radius
		dLon = y_meters/(earth_radius*math.cos(math.pi*self.__gcs_cmd_home_lat/180))
		#Converting from radians to degrees
		dLat = dLat * 180/math.pi
		dLon = dLon * 180/math.pi

		return dLat, dLon


	#Drone goes to initial location as commanded by GCS
	# (Can be made as protected function later)
	def go_to_location(self):
		
		#Setup for initial sorty
		if math.isnan(self.__gcs_cmd_x_diff):
			self.__gcs_cmd_x_diff = 0.0
		if math.isnan(self.__gcs_cmd_y_diff):
			self.__gcs_cmd_y_diff = 0.0
		if math.isnan(self.__gcs_cmd_x):
			self.__gcs_cmd_x = 0.0
		if math.isnan(self.__gcs_cmd_y):
			self.__gcs_cmd_y = 0.0
		if math.isnan(self.__gcs_cmd_height):
			self.__gcs_cmd_height = self.toh
		if math.isnan(self.__gcs_cmd_yaw):
			self.__gcs_cmd_yaw = self.__cur_yaw
		
		#Publisher function to go to the desired initial location
		pos_pub = rospy.Publisher(self.drone_ID+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		init_pos = PoseStamped()
		init_pos.header.stamp = rospy.Time.now()
		init_pos.pose.position.x = self.__gcs_cmd_x + self.__gcs_cmd_x_diff
		init_pos.pose.position.y = self.__gcs_cmd_y + self.__gcs_cmd_y_diff
		init_pos.pose.position.z = self.__gcs_cmd_height
		[init_pos.pose.orientation.x, init_pos.pose.orientation.y, init_pos.pose.orientation.z, init_pos.pose.orientation.w] = quaternion_from_euler(0,0,self.__gcs_cmd_yaw)


		if not self.__armed:

			if self.__bat>=safe:

				#For arm and takeoff
				mavros.command.arming(True)
				
				time.sleep(1)
				
				takeoff_alt = ParamValue()
				takeoff_alt.real = self.toh
				self.__set_param(param_id='MIS_TAKEOFF_ALT', value=takeoff_alt) 

				self.__set_mode(0, 'AUTO.TAKEOFF')

				while(self.__cur_alt <= 0.9*self.toh):
					time.sleep(1)

				#Go to initial location as commanded by GCS
				pos_pub.publish(init_pos)
				self.__set_mode(0, 'OFFBOARD')

				while((abs(self.__cur_x-self.__gcs_cmd_x)>=1) or (abs(self.__cur_y-self.__gcs_cmd_y)>=1)):			
					pos_pub.publish(init_pos)
					time.sleep(0.2)

				self.__set_mode(0, 'AUTO.LOITER')
					

			#if battery level not sufficient
			else:
				print(self.drone_ID + ' battery level below safe level. Therefore cannot arm and takeoff')
				exit()
		
		#if drone already armed 
		else:
			print(self.drone_ID + ' already armed. Hence cannont takeoff')
			self.__set_mode(0, 'AUTO.LAND')
			while self.__armed:
				time.sleep(1)
			exit()
			

	#Surveillance Mission
	# (Can be made as protected function later)
	def normal_mission(self):

		self.__dLat, self.__dLon = [None]*len(self.WP), [None]*len(self.WP)

		#Function to convert distance (in meters) to latitude and longitude
		for i in range(len(self.WP)):
			self.__dLat[i], self.__dLon[i] = self.__dis_to_gps(self.WP[i][0],self.WP[i][1])

		#Setting ground speed
		mission_speed = ParamValue()
		mission_speed.real = 1.0
		self.__set_param(param_id='MPC_XY_VEL_MAX', value=mission_speed)

		#Create waypoints
		W = self.__waypoint_dataset()

		time.sleep(0.5)

		#Uploading waypoints and mission
		self.__set_waypoint(start_index=0, waypoints=W)

		time.sleep(0.5)

		#Subscriber function to make mission continuous
		rospy.Subscriber(self.drone_ID+'/mavros/mission/waypoints', WaypointList, self.__control_mission_cb)

		#set current waypoint to gcs_cmd_waypoint
		self.__set_cur_waypoint(wp_seq=self.__gcs_cmd_wp)
		
		#For doing mission
		if self.__armed:
			self.__set_mode(0, 'AUTO.MISSION')
			while(self.__bat>=low):
				continue
			self.__set_mode(0, 'AUTO.LOITER')

		#if drone not armed 
		else:
			print(self.drone_ID + ' cannont continue on mission. Please check')
			exit()	


	#Sequence to wait for replacement drone
	# (Can be made as protected function later)
	def hower_and_wait(self):
			
		#Publisher to increase drone's altitude
		alt_pub = rospy.Publisher(self.drone_ID+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		inc_alt = PoseStamped()
		inc_alt.header.stamp = rospy.Time.now()
		inc_alt.pose.position.x = self.__gcs_cmd_x + self.__gcs_cmd_x_diff
		inc_alt.pose.position.y = self.__gcs_cmd_y + self.__gcs_cmd_y_diff
		inc_alt.pose.position.z = self.__gcs_cmd_height
		[inc_alt.pose.orientation.x, inc_alt.pose.orientation.y, inc_alt.pose.orientation.z, inc_alt.pose.orientation.w] = quaternion_from_euler(0,0,self.__gcs_cmd_yaw)
		
		#Increase altitude and hover
		if self.__armed:
			alt_pub.publish(inc_alt)
			self.__set_mode(0, 'OFFBOARD')
			while(self.__cur_alt <= 0.95*self.__gcs_cmd_height):
				alt_pub.publish(inc_alt)
			self.__set_mode(0, 'AUTO.LOITER')

		#if drone not armed 
		else:
			print(self.drone_ID + ' not armed. Cannot initiate drone substitution')
			exit()


	#Land sequence using Aruco Marker
	# (Can be made as protected function later)
	def go_to_home(self):

		'''
		if self.__armed:
			land_obj = LandUsingMPC(drone_ID=self.drone_ID, aruco_lat=self.__gcs_cmd_aruco_lat, aruco_lon=self.__gcs_cmd_aruco_lon, home_alt=self.__gcs_cmd_aruco_alt, call=True, first_land=self.__first_land)
			self.__pub_aruco.publish(True)
			land_obj.land()
			self.__pub_aruco.publish(False)

		else:
			print(self.drone_ID + ' not armed')
			exit()
		
		if self.__first_land:
			self.__first_land = False
		'''

		# Simple land sequence (without MPC) for testing on simulation only
		
		pub_goto = rospy.Publisher(self.drone_ID+'/mavros/setpoint_position/global',  GeoPoseStamped, queue_size=10)
		gl = GeoPoseStamped()
		gl.header.stamp = rospy.Time.now()
		gl.pose.position.latitude = self.__gcs_cmd_aruco_lat
		gl.pose.position.longitude = self.__gcs_cmd_aruco_lon
		gl.pose.position.altitude = self.__gcs_cmd_aruco_alt + 5
		[gl.pose.orientation.x, gl.pose.orientation.y, gl.pose.orientation.z, gl.pose.orientation.w] = quaternion_from_euler(0,0,0)

		#Increase altitude and hover
		if self.__armed:
			pub_goto.publish(gl)
			for i in range(2000):
				self.__set_mode(0, 'OFFBOARD')
				pub_goto.publish(gl)
			self.__set_mode(0, 'AUTO.LAND')


	#Function to subscribe to the GCS command and take action accordingly
	# (Can be made as protected function later)
	def drone_sub(self):

		#Subscriber function to get GCS commands
		rospy.Subscriber(drone_ID+'_gcs_command', gcs_msg, self.__gcs_command_cb)

		#Subscriber functions
		rospy.Subscriber(self.drone_ID+'/mavros/state', State, self.__armed_cb)
		rospy.Subscriber(self.drone_ID+'/mavros/global_position/local', Odometry, self.__location_cb)
		rospy.Subscriber(self.drone_ID+'/mavros/imu/data', Imu, self.__ori_cb)

		print(drone_ID + ' Ready')
		

		while not rospy.is_shutdown():

			if self.__gcs_cmd_str == self.drone_ID + ' takeoff': 		#GCS takeoff and go to position command for drone			
				print(self.drone_ID + ' copy takeoff')
				self.__pub_status.publish(self.drone_ID + ' takeoff')
				self.go_to_location()
				self.__pub_status.publish(self.drone_ID + ' at location')

			if self.__gcs_cmd_str == self.drone_ID + ' do mission':			#GCS normal mission command for drone
				print(self.drone_ID + ' copy doing mission')
				self.__pub_status.publish(self.drone_ID + ' doing mission')
				self.normal_mission()
				self.__pub_status.publish(self.drone_ID + ' need replacement')
			
			if self.__gcs_cmd_str == self.drone_ID + ' replacement ready':			#GCS command for drone to suggest that another drone is available for replacement
				print(self.drone_ID + ' copy preparing for replacement')
				self.__pub_status.publish(self.drone_ID + ' copy preparing for replacement')
				self.hower_and_wait()
				self.__pub_status.publish(self.drone_ID + ' ready for replacement')
			
			if self.__gcs_cmd_str == self.drone_ID + ' land':					#GCS go to home location and land command for drone
				print(self.drone_ID + ' copy land')
				self.__pub_status.publish(self.drone_ID + ' copy landing')
				self.go_to_home()
				self.__pub_status.publish(self.drone_ID + ' grounded')
			
			if self.__gcs_cmd_str == 'replacement for ' + self.drone_ID + ' not ready':	#GCS command for drone to suggest that no other drone is available for replacement
				print(self.drone_ID + ' copy error')
				self.__set_mode(0, 'AUTO.LAND')
				while self.__armed:
					time.sleep(1)
				exit()
			
			rospy.sleep(0.1)



	#Function to get battery level
	# (Can be made as protected function later)
	def drone_bat(self):
		rospy.Subscriber(self.drone_ID+'/mavros/battery', BatteryState, self.__bat_cb)

	
	#-------------------
	#FOR SIMULATION ONLY
	#-------------------

	#Function to simulate the charging and discharging of battery level
	# (Can be made as protected function later)
	def drone_bat_sim(self):

		pub_bat = rospy.Publisher(self.drone_ID+'_battery', Int16, queue_size=1)

		while not rospy.is_shutdown():
			
			if self.__armed:
				if self.__bat>0:
					time.sleep(75)
					self.__bat = self.__bat - 100
				else:
					self.__bat = 0
			if not self.__armed:
				if self.__bat<100:
					time.sleep(10)
					self.__bat = self.__bat + 10
				else:
					self.__bat = 100

			pub_bat.publish(self.__bat)

			rospy.sleep(0.1)


	#main executable function
	def run(self):

		thread1 = threading.Thread(target=self.drone_sub)

		if self.Simulation:
			thread2 = threading.Thread(target=self.drone_bat_sim)
		else:
			thread2 = threading.Thread(target=self.drone_bat)

		thread1.start()
		thread2.daemon=True
		thread2.start()

		thread1.join()
		thread2.join()


if __name__ == '__main__':

########################################################################################################
#
#	    W[1]								W[1]
#	W[4]*____________________________________* W[5]	    	
#		|				   {				 |
#		|				   {				 |				North (+x direction)
#		|				   { x_lat			 |					^		
#		|				   { 	    		 |					|
#		|				   {	y_long		 |					|
#		|			   W[0]*<--------------->|					|
#		|				 home				 |					|----------> East (+y direction)
#		|									 |
#		|									 |
#		|									 |
#		*____________________________________*
#	W[3]									W[2]
#
########################################################################################################

	#parser so that arguments can be ammended during run time
	parser = argparse.ArgumentParser(description='drone_params')
	parser.add_argument('--dr_no', default=0, type=int)
	parser.add_argument('--toh', default=5.0, type=float)				#Desired Takeoff height for mission
	
	#Mission waypoints in (x,y,heading_angle)
	parser.add_argument('--waypoints', default=((5,5,0),				#Coordinate for W[1]/W[5] and heading angle while going to W[1]/W[5]
												(-5,5,90),				#Coordinate for W[2] and heading angle while going to W[2]
												(-5,-5,180),			#Coordinate for W[3] and heading angle while going to W[3]
												(5,-5,270)),			#Coordinate for W[4] and heading angle while going to W[4]		
												nargs="+",
												type=list)

	parser.add_argument('--safe_bat',default=90, type=int)				#Minimum safe battery level to authorize takeoff (percentage)
	parser.add_argument('--low_bat', default=80, type=int)				#Low battery level (percentage)
	parser.add_argument('--critical_bat', default=70, type=int)			#Critical battery level (percentage)
	parser.add_argument('--Simulation', default=True, type=bool)

	args = parser.parse_args()

	#Making sure that drone number is valid
	if args.dr_no<=0:
		print('Drone number invalid')
		exit()

	#Making sure all battery levels are at most 100%
	if args.safe_bat>100:
		print('Safe battery level is set to over 100%')
		exit()
	if args.low_bat>100:
		print('Low battery level is set to over 100%')
		exit()
	if args.critical_bat>100:
		print('Critical battery level is set to over 100%')
		exit()

	#Making sure that low battery level is sufficiently below safe battery level
	if args.safe_bat-args.low_bat<10:
		print('Make sure low battery level is set to at least 20% below safe battery level')
		exit()

	#Making sure that critical battery level is sufficiently below low battery level
	if args.low_bat-args.critical_bat<10:
		print('Make sure critical battery level is set to at least 20% below low battery level')
		exit()

	drone_num = args.dr_no					
	toh = args.toh							
	WP = args.waypoints		
	safe = args.safe_bat						
	low = args.low_bat 							
	critical = args.critical_bat
	Simulation = args.Simulation			

	#setting minimum takeoff height (min_toh = 5 meters) for safety
	if toh<5:
		toh=5

	rospy.init_node('drone_'+str(drone_num))

	#ID of drone (Ex. drone1 , drone2 , drone3 ...)
	drone_ID = 'drone' + str(drone_num)

	#setting up mavros services
	mavros.set_namespace(drone_ID+'/mavros')

	drone = DroneMission(drone_ID=drone_ID, toh=toh, WP=WP, safe=safe, low=low, critical=critical, Simulation=Simulation)
	drone.run()
	
