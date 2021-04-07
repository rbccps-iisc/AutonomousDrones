#!/usr/bin/env python

# Code to control drone
# running this code should cause drone to take commands from GCS and act accordingly
# Usage - python drone_mission.py 


from os import sys, path

sys.path.append('../MPC_lander')
import drone_control

sys.path.append('../opencv')
from lib_aruco_pose import ArucoSingleTracker

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
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry

from multidrone_mission.msg import gcs_msg


# battery percentage (for simulation only)
bat = 100
# drone flight status
status = 'not yet initiated'
# drone armed status
armed = False

#To detect first time landing
first_land = True

# Command from GSC
gcs_cmd_home_lat = float('nan')
gcs_cmd_home_lon = float('nan')
gcs_cmd_home_alt = float('nan')
gcs_cmd_x = float('nan')
gcs_cmd_y = float('nan')
gcs_cmd_yaw = float('nan')
gcs_cmd_wp = 1

#Current reletive location
cur_x = 0
cur_y = 0
cur_alt = 0

#Current reletive attitude
cur_roll = 0
cur_pitch = 0
cur_yaw = 0

#Current waypoint
cur_wp = 1


#Function callback for drone_ID+'/mavros/state'
def armed_cb(data):
	global armed
	armed = data.armed

#Function callback for drone_ID+'/mavros/battery'
def bat_cb(data):
	global bat
	bat = data.percentage

#Function callback for drone_ID+'/mavros/global_position/local'
def location_cb(data):
	global cur_x,cur_y,cur_alt
	cur_x = data.pose.pose.position.x
	cur_y = data.pose.pose.position.y
	cur_alt = data.pose.pose.position.z

#Function callback for drone_ID+'/mavros/global_position/global'
def home_cb(data):
	global gcs_cmd_home_lat, gcs_cmd_home_lon, home_pos
	gcs_cmd_home_lat = data.latitude
	gcs_cmd_home_lon = data.longitude
	home_pos.unregister()

def alt_cb(data):
	global gcs_cmd_home_alt, home_alt
	gcs_cmd_home_alt = data.amsl
	home_alt.unregister()

#Function callback for drone_ID+'/mavros/imu/data'
def ori_cb(data):
	global cur_roll,cur_pitch,cur_yaw
	orientation_q = data.orientation
	orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
	(cur_roll, cur_pitch, cur_yaw) = euler_from_quaternion(orientation_list)


#Function callback for drone_ID+'/mavros/mission/waypoints'
def control_mission_cb(data, args):
	global cur_wp
	cur_wp = data.current_seq
	set_cur_waypoint = args[0]
	WP = args[1]

	#W[last]==W[first] (Required to keep mission continued)
	if cur_wp==(len(WP)+1):
		set_cur_waypoint(wp_seq=1)
		cur_wp = 1


#Function called by normal_mission()
def waypoint_dataset(dLat,dLon,WP,toh):
	global gcs_cmd_home_lat, gcs_cmd_home_lon

	W = []
	WP_number = len(WP)

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 22  		#Takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 1 					#delay 
	wp.param2 = 1					#Accept Radius
	wp.param3 = 1					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat		#Latitude
	wp.y_long = gcs_cmd_home_lon	#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)

	for i in range(WP_number):	
		wp = Waypoint()
		wp.frame = 3			#FRAME_GLOBAL_REL_ALT
		wp.command = 16  		#Navigate to waypoint.
		wp.is_current = False
		wp.autocontinue = True
		wp.param1 = 1  								#delay
		wp.param2 = 1								#Accept Radius
		wp.param3 = 1								#Pass Radius
		wp.param4 = WP[i][2]						#Yaw
		wp.x_lat = gcs_cmd_home_lat + dLat[i]		#Latitude
		wp.y_long = gcs_cmd_home_lon + dLon[i]		#Longitude
		wp.z_alt = toh								#altitude
		W.append(wp)

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 16  		#Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 1  								#delay
	wp.param2 = 1								#Accept Radius
	wp.param3 = 1								#Pass Radius
	wp.param4 = WP[0][2]						#Yaw
	wp.x_lat = gcs_cmd_home_lat + dLat[0]		#Latitude
	wp.y_long = gcs_cmd_home_lon + dLon[0]		#Longitude
	wp.z_alt = toh								#altitude
	W.append(wp)
	
	return(W)

#Function called by normal_mission()
def dis_to_gps(x_meters,y_meters):
	global gcs_cmd_home_lat

	earth_radius=6378137.0 #Radius of "spherical" earth
	#Converting desired trajectories of drone in meters to GPS coordinates (in radians)
	dLat = x_meters/earth_radius
	dLon = y_meters/(earth_radius*math.cos(math.pi*gcs_cmd_home_lat/180))
	#Converting from radians to degrees
	dLat = dLat * 180/math.pi
	dLon = dLon * 180/math.pi

	return dLat, dLon


#Function called by drone_sub()
def go_to_location(drone_ID,set_mode,set_param,toh,WP,safe):
	global bat, armed, cur_x, cur_y, cur_alt, cur_yaw, gcs_cmd_x, gcs_cmd_y, gcs_cmd_yaw

	#setting up mavros services
	mavros.set_namespace(drone_ID+'/mavros')

	#Subscriber function to get current altitude of drone
	rospy.Subscriber(drone_ID+'/mavros/global_position/local', Odometry, location_cb)
	#Subscriber function to get current attitude of drone
	rospy.Subscriber(drone_ID+'/mavros/imu/data', Imu, ori_cb)

	#time.sleep(1)
	
	#Setup for initial sorty
	if math.isnan(gcs_cmd_x):
		gcs_cmd_x = 0.0
	if math.isnan(gcs_cmd_y):
		gcs_cmd_y = 0.0
	if math.isnan(gcs_cmd_yaw):
		gcs_cmd_yaw = cur_yaw
	
	#Publisher function to go to the desired initial location
	pos_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_position/local', PoseStamped, queue_size=1)
	init_pos = PoseStamped()
	# init_pos.header.stamp = rospy.Time.now()
	init_pos.pose.position.x = gcs_cmd_x
	init_pos.pose.position.y = gcs_cmd_y
	init_pos.pose.position.z = toh
	[init_pos.pose.orientation.x, init_pos.pose.orientation.y, init_pos.pose.orientation.z, init_pos.pose.orientation.w] = quaternion_from_euler(0,0,gcs_cmd_yaw)

	if not armed:

		if bat>=safe:

			#For arm and takeoff
			#set_mode(custom_mode='GUIDED')

			mavros.command.arming(True)
			time.sleep(1)
			
			takeoff_alt = ParamValue()
			takeoff_alt.real = toh
			set_param(param_id='MIS_TAKEOFF_ALT', value=takeoff_alt) 

			set_mode(0, 'AUTO.TAKEOFF')
			while(cur_alt <= 0.9*toh):
				time.sleep(1)

			#Go to initial location as commanded by GCS
			pos_pub.publish(init_pos)
			set_mode(0, 'OFFBOARD')
			pos_pub.publish(init_pos)

			while((abs(cur_x-gcs_cmd_x)>=1) or (abs(cur_y-gcs_cmd_y)>=1)):
				#print(abs(cur_x-gcs_cmd_x), abs(cur_y-gcs_cmd_y))
				pos_pub.publish(init_pos)
				time.sleep(0.2)
				
			set_mode(0, 'AUTO.LOITER')

		#if battery level not sufficient
		else:
			print(drone_ID + ' battery level below safe level. Therefore cannot arm and takeoff')
			exit()
	
	#if drone already armed 
	else:
		print(drone_ID + ' already armed. Hence cannont takeoff')
		set_mode(0, 'AUTO.LAND')
		while armed:
			time.sleep(1)
		exit()
		

#Function called by drone_sub()
def normal_mission(drone_ID,waypoints_clean,set_waypoint,set_cur_waypoint,set_mode,set_param,WP,toh,low,critical):
	global bat, armed, gcs_cmd_x, gcs_cmd_y, gcs_cmd_yaw, gcs_cmd_wp, cur_alt, cur_x, cur_y, cur_yaw, cur_wp

	dLat, dLon = [None]*len(WP), [None]*len(WP)

	#Function to convert distance (in meters) to latitude and longitude
	for i in range(len(WP)):
		dLat[i], dLon[i] = dis_to_gps(WP[i][0],WP[i][1])

	# waypoints_clean.call()

	#Setting ground speed
	mission_speed = ParamValue()
	mission_speed.real = 2.0
	set_param(param_id='MPC_XY_VEL_MAX', value=mission_speed)

	#Create waypoints
	W = waypoint_dataset(dLat,dLon,WP,toh)

	time.sleep(0.5)

	#Uploading waypoints and mission
	set_waypoint(start_index=0, waypoints=W)

	time.sleep(0.5)

	#Subscriber function to control mission parameters
	rospy.Subscriber(drone_ID+'/mavros/mission/waypoints', WaypointList, control_mission_cb, (set_cur_waypoint,WP))

	#set current waypoint to gcs_cmd_waypoint
	set_cur_waypoint(wp_seq=gcs_cmd_wp)
	
	#For doing mission
	if armed:
		set_mode(0, 'AUTO.MISSION')
		while(bat>=low):
			continue
		gcs_cmd_x = cur_x
		gcs_cmd_y = cur_y
		gcs_cmd_yaw = cur_yaw
		gcs_cmd_wp = cur_wp
		set_mode(0, 'AUTO.LOITER')

	else:
		print(drone_ID + ' cannont continue on mission. Please check')
		exit()	


#Function called by drone_sub()
def go_to_home(drone_ID,pub_aruco):
	global armed, gcs_cmd_home_lat, gcs_cmd_home_lon, gcs_cmd_home_alt, first_land

	if armed:
		pub_aruco.publish(True)
		drone_control.main(drone_ID=drone_ID, home_lat=gcs_cmd_home_lat, home_lon=gcs_cmd_home_lon, home_alt=gcs_cmd_home_alt, call=True, first_land=first_land)
		pub_aruco.publish(False)

	else:
		print(drone_ID + ' not armed')
		exit()
	
	if first_land:
		first_land = False


#Function to subscribe to the GCS command and take action accordingly (runs on sepatate thread)
def drone_sub(drone_ID,toh,WP,safe,low,critical):
	global status, armed, bat, cur_alt, home_pos, home_alt

	# Definitions for rospy services
	set_mode = rospy.ServiceProxy(drone_ID+'/mavros/set_mode', SetMode)
	#set_takeoff = rospy.ServiceProxy(drone_ID+'/mavros/cmd/takeoff', CommandTOL)
	#set_land = rospy.ServiceProxy(drone_ID+'/mavros/cmd/land', CommandTOL)
	set_cur_waypoint = rospy.ServiceProxy(drone_ID+'/mavros/mission/set_current', WaypointSetCurrent)
	#set_cmd = rospy.ServiceProxy(drone_ID+'/mavros/cmd/command', CommandLong)
	waypoints_clean = rospy.ServiceProxy(drone_ID+'/mavros/mission/clear', WaypointClear)
	set_waypoint = rospy.ServiceProxy(drone_ID+'/mavros/mission/push', WaypointPush)
	set_param = rospy.ServiceProxy(drone_ID+'/mavros/param/set',ParamSet)


	#Subscriber to get home lattitude and longitude
	home_pos = rospy.Subscriber(drone_ID+'/mavros/global_position/global', NavSatFix, home_cb)

	#Subscriber to get home altitude
	home_alt = rospy.Subscriber(drone_ID+'/mavros/altitude', Altitude, alt_cb)

	#Publisher function to publish status of drone
	pub_status = rospy.Publisher(drone_ID+'_status', String, queue_size=1)

	#Publisher function to publish whether or not to run aruco detection code
	pub_aruco = rospy.Publisher('run_aruco_detect', Bool, queue_size=1)
	
	
	print(drone_ID + ' Ready')
	
	while not rospy.is_shutdown():

		if not armed and bat>=safe: 			#GCS takeoff and go to position command for drone			
			print(drone_ID + ' copy takeoff')
			pub_status.publish(drone_ID + ' takeoff')
			go_to_location(drone_ID,set_mode,set_param,toh,WP,safe)
			pub_status.publish(drone_ID + ' at location')
			print(drone_ID + ' copy doing mission')
			pub_status.publish(drone_ID + ' doing mission')
			normal_mission(drone_ID,waypoints_clean,set_waypoint,set_cur_waypoint,set_mode,set_param,WP,toh,low,critical)
			pub_status.publish(drone_ID + ' need replacement')
		
		if armed and bat<=low:					#GCS go to home location and land command for drone
			print(drone_ID + ' copy land')
			pub_status.publish(drone_ID + ' copy landing')
			go_to_home(drone_ID,pub_aruco)
			pub_status.publish(drone_ID + ' grounded')
		
		rospy.sleep(0.1)



#Function to simulate and publish charging and discharging of battery level (runs on sepatate thread)
def drone_bat(drone_ID):
	global bat
	rospy.Subscriber(drone_ID+'/mavros/battery', BatteryState, bat_cb)



#FOR SIMULATION ONLY
#Function to simulate and publish charging and discharging of battery level (runs on sepatate thread)
def drone_bat_sim(drone_ID):
	global bat, status, armed

	#Subscriber function to check if drone is armed or not 
	rospy.Subscriber(drone_ID+'/mavros/state', State, armed_cb)

	#pub_status = rospy.Publisher(drone_ID+'_status', String, queue_size=1)
	pub_bat = rospy.Publisher(drone_ID+'_battery', Int16, queue_size=1)
	
	while not rospy.is_shutdown():
		
		if armed:
			if bat>0:

				time.sleep(20)
				bat = bat - 100
			else:
				bat = 0
		if not armed:
			if bat<100:
				time.sleep(15)

				bat = bat + 100
			else:
				bat = 100

		pub_bat.publish(bat)
		#print(drone_ID + ' battery percent = %d' %(bat))
		#print(drone_ID + ' status = %s' %(status))
		rospy.sleep(0.1)



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
	parser = argparse.ArgumentParser(description='params')
	parser.add_argument('--dr_no', default=1, type=int)
	parser.add_argument('--toh', default=10.0, type=float)				#Desired Takeoff height for mission
	
	#Mission waypoints in (x,y,heading_angle)
	parser.add_argument('--waypoints', default=((5,5,0),				#Coordinate for W[1]/W[5] and heading angle while going to W[1]/W[5]
												(-5,5,90),				#Coordinate for W[2] and heading angle while going to W[2]
												(-5,-5,180),			#Coordinate for W[3] and heading angle while going to W[3]
												(5,-5,270)))			#Coordinate for W[4] and heading angle while going to W[4]		

	parser.add_argument('--safe_bat',default=90, type=int)				#Minimum safe battery level to authorize takeoff (percentage)
	parser.add_argument('--low_bat', default=80, type=int)				#Low battery level (percentage)
	parser.add_argument('--critical_bat', default=70, type=int)			#Critical battery level (percentage)
	parser.add_argument('--Simulation', default=True, type=bool)

	args = parser.parse_args()

	#Making sure that these params are checked before operation
	if args.dr_no<=0:
		print('Drone number entered already exists')
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

	#setting minimum takeoff height (min_toh = 8 meters) for safety
	if toh<5:
		toh=5

	rospy.init_node('drone_'+str(drone_num))

	#ID of drone (Ex. drone1 , drone2 , drone3 ...) 
	drone_ID = 'drone' + str(drone_num)

	thread1 = threading.Thread(target=drone_sub,args=(drone_ID,toh,WP,safe,low,critical))

	if Simulation:
		thread2 = threading.Thread(target=drone_bat_sim,args=(drone_ID,))
	else:
		thread2 = threading.Thread(target=drone_bat,args=(drone_ID,))

	thread1.start()
	thread2.daemon=True
	thread2.start()
