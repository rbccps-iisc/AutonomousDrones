#!/usr/bin/env python3

# Code to control drone
# running this code should cause drone to take commands from GCS and act accordingly
# Usage - python drone_mission.py


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import argparse
import threading
import time, math

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Header

import rospy, mavros
from mavros import command

from std_msgs.msg import String, Int16
#from mavros_msgs.msg import State
#from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry

from multidrone_mission.msg import gcs_msg


# battery percentage
bat = 100
# drone flight status
status = 'not yet initiated'
# drone armed status
armed = False

# Command from GSC
gcs_cmd_str = 'none'
gcs_cmd_home_lat = float('nan')
gcs_cmd_home_lon = float('nan')
gcs_cmd_x_diff = float('nan')
gcs_cmd_y_diff = float('nan')
gcs_cmd_x = float('nan')
gcs_cmd_y = float('nan')
gcs_cmd_height = float('nan')
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


#Function callback for drone_ID+'/mavros/state'
def armed_cb(data):
	global armed
	armed = data.armed

#Function callback for drone_ID+'/mavros/global_position/local'
def location_cb(data):
	global cur_x,cur_y,cur_alt
	cur_x = data.pose.pose.position.x
	cur_y = data.pose.pose.position.y
	cur_alt = data.pose.pose.position.z

#Function callback for drone_ID+'/mavros/imu/data'
def ori_cb(data):
	global cur_roll,cur_pitch,cur_yaw
	orientation_q = data.orientation
	orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
	(cur_roll, cur_pitch, cur_yaw) = euler_from_quaternion(orientation_list)

#Function callback for 'gcs_command'
def gcs_command_cb(data):
	global gcs_cmd_str, gcs_cmd_home_lat, gcs_cmd_home_lon, gcs_cmd_x_diff, gcs_cmd_y_diff, gcs_cmd_x, gcs_cmd_y, gcs_cmd_height, gcs_cmd_yaw, gcs_cmd_wp
	gcs_cmd_str = data.command
	gcs_cmd_home_lat = data.home_lat
	gcs_cmd_home_lon = data.home_lon
	gcs_cmd_x_diff = data.x_diff
	gcs_cmd_y_diff = data.y_diff
	gcs_cmd_x = data.x
	gcs_cmd_y = data.y
	gcs_cmd_height = data.height
	gcs_cmd_yaw = data.yaw 	
	gcs_cmd_wp = data.waypoint


#Function callback for drone_ID+'/mavros/mission/waypoints'
def control_mission_cb(data, args):
	cur_wp = data.current_seq
	set_cmd = args[0]
	set_cur_waypoint = args[1]

	#command = 178 (for setting ground speed) ; param2 = speed (m/s)
	if cur_wp!=0:
		set_cmd(command=178, param2=0.5)

	#command = 115 (for setting yaw) ; param1 = yaw angle (deg) ; param2 = yaw speed (deg/s) ; param4 = 0 (global frame)
	if cur_wp==2:
		set_cmd(command=115, param1=0, param2=45, param4=0)		# W[1] to W[2] (Refer to below diageam) (param1=0 -> Facing North)
	if cur_wp==3:
		set_cmd(command=115, param1=90, param2=45, param4=0)	# W[2] to W[3] (Refer to below diageam) (param1=90 -> Facing East)
	if cur_wp==4:
		set_cmd(command=115, param1=180, param2=45, param4=0)	# W[3] to W[4] (Refer to below diageam) (param1=180 -> Facing South)
	if cur_wp==5:
		set_cur_waypoint(wp_seq=1)								# W[5]==W[1] (Refer to below diageam) (Required to keep mission continued)
	if cur_wp==1:
		set_cmd(command=115, param1=270, param2=45, param4=0)	# W[4] to W[5]/W[1] (Refer to below diageam) (param1=270 -> Facing West)


#Function called by normal_mission()
def waypoint_dataset(dLat,dLon,toh):

########################################################################################################
#
#	    W[1]								W[2]
#	W[5]*____________________________________*		    	
#		|				   {				 |			  North (+ x_lat)
#		|				   {				 |				^
#		|				   { x_lat			 |				|
#		|				   { 	    		 |				|
#		|		y_long	   {				 |				|
#		|<---------------->* W[0]			 |				|----------> East (+ y_long)
#		|				 home				 |
#		|									 |
#		|									 |
#		|									 |
#		*____________________________________*
#	W[4]									W[3]
#
# This function is used to set the waypoint array (W[]) according to the diagram. Function returns W[]
########################################################################################################

	global gcs_cmd_home_lat, gcs_cmd_home_lon

	W = []

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 22  		#Takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0 					#delay 
	wp.param2 = 0					#Accept Radius
	wp.param3 = 0					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat				#Latitude
	wp.y_long = gcs_cmd_home_lon			#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)	

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 16  		#Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  					#delay (must correspond to param2 in control_mission_cb())
	wp.param2 = 0					#Accept Radius
	wp.param3 = 0					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat + dLat		#Latitude
	wp.y_long = gcs_cmd_home_lon - dLon		#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 16  		#Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  					#delay (must correspond to param2 in control_mission_cb())
	wp.param2 = 0					#Accept Radius
	wp.param3 = 0					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat + dLat		#Latitude
	wp.y_long = gcs_cmd_home_lon + dLon		#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)
	
	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 16  		#Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  					#delay (must correspond to param2 in control_mission_cb())
	wp.param2 = 0					#Accept Radius
	wp.param3 = 0					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat - dLat		#Latitude
	wp.y_long = gcs_cmd_home_lon + dLon		#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 16  		#Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0					#delay (must correspond to param2 in control_mission_cb())
	wp.param2 = 0					#Accept Radius
	wp.param3 = 0					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat - dLat		#Latitude
	wp.y_long = gcs_cmd_home_lon - dLon		#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)

	wp = Waypoint()
	wp.frame = 3			#FRAME_GLOBAL_REL_ALT
	wp.command = 16  		#Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  					#delay
	wp.param2 = 0					#Accept Radius
	wp.param3 = 0					#Pass Radius
	wp.param4 = 0					#Yaw
	wp.x_lat = gcs_cmd_home_lat + dLat		#Latitude
	wp.y_long = gcs_cmd_home_lon - dLon		#Longitude
	wp.z_alt = toh					#altitude
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

	return (dLat, dLon)


#Function called by drone_sub()
def go_to_location(drone_ID,set_mode,set_takeoff,set_land,toh,x_meters,y_meters,safe):
	global bat, armed, cur_x, cur_y, cur_alt, cur_yaw, gcs_cmd_x_diff, gcs_cmd_y_diff, gcs_cmd_x, gcs_cmd_y, gcs_cmd_height, gcs_cmd_yaw

	#setting up mavros services
	mavros.set_namespace(drone_ID+'/mavros')

	#Subscriber function to get current altitude of drone
	rospy.Subscriber(drone_ID+'/mavros/global_position/local', Odometry, location_cb)
	#Subscriber function to get current attitude of drone
	rospy.Subscriber(drone_ID+'/mavros/imu/data', Imu, ori_cb)

	time.sleep(1)
	
	#Setup for initial sorty
	if math.isnan(gcs_cmd_x_diff):
		gcs_cmd_x_diff = 0
	if math.isnan(gcs_cmd_y_diff):
		gcs_cmd_y_diff = 0
	if math.isnan(gcs_cmd_x):
		gcs_cmd_x = -x_meters
	if math.isnan(gcs_cmd_y):
		gcs_cmd_y = y_meters
	if math.isnan(gcs_cmd_height):
		gcs_cmd_height = toh
	if math.isnan(gcs_cmd_yaw):
		gcs_cmd_yaw = cur_yaw
	
	
	#Publisher function to go to the desired initial location
	pos_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	init_pos = PoseStamped()
	init_pos.header.stamp = rospy.Time.now()
	init_pos.pose.position.x = gcs_cmd_x + gcs_cmd_x_diff
	init_pos.pose.position.y = gcs_cmd_y + gcs_cmd_y_diff
	init_pos.pose.position.z = gcs_cmd_height
	[init_pos.pose.orientation.x, init_pos.pose.orientation.y, init_pos.pose.orientation.z, init_pos.pose.orientation.w] = quaternion_from_euler(0,0,gcs_cmd_yaw)
	
	'''
	pos_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
	init_pos = PositionTarget()
	init_pos.header.stamp = rospy.Time.now()
	init_pos.coordinate_frame = 1
	init_pos.type_mask = 0b001111000000
	init_pos.position.x = gcs_cmd_x + gcs_cmd_x_diff
	init_pos.position.y = gcs_cmd_y + gcs_cmd_y_diff
	init_pos.position.z = gcs_cmd_height
	init_pos.velocity.x = 0.5
	init_pos.velocity.y = 0.5
	init_pos.velocity.z = 0.3
	init_pos.yaw = gcs_cmd_yaw
	init_pos.yaw_rate = 20
	'''

	if not armed:

		if bat>=safe:

			#For arm and takeoff
			set_mode(custom_mode='GUIDED')

			mavros.command.arming(True)
			time.sleep(1)

			set_takeoff(0, 0, None, None, toh)
			while(cur_alt <= 0.95*toh):
				time.sleep(1)

			#Go to initial location as commanded by GCS
			pos_pub.publish(init_pos)
			while((abs(cur_x-gcs_cmd_x)>=0.2) or (abs(cur_y-gcs_cmd_y)>=0.2)):
				#print(abs(cur_x-gcs_cmd_x) , abs(cur_y-gcs_cmd_y))
				time.sleep(1)

		#if battery level not sufficient
		else:
			print(drone_ID + ' battery level below safe level. Therefore cannot arm and takeoff')
			exit()
	
	#if drone already armed 
	else:
		print(drone_ID + ' already armed. Hence cannont takeoff')
		set_land(0, 0, None, None, 0)
		while armed:
			time.sleep(1)
		exit()
		

#Function called by drone_sub()
def normal_mission(drone_ID,waypoints_clean,set_waypoint,set_cur_waypoint,set_cmd,set_mode,x_meters,y_meters,toh,low,critical):
	global bat, armed, gcs_cmd_wp, cut_alt

	#Function to convert distance (in meters) to latitude and longitude
	dLat, dLon = dis_to_gps(x_meters,y_meters)

	waypoints_clean.call()

	#Create waypoints for function by calling waypoint_dataset() [defined above]
	W = waypoint_dataset(dLat,dLon,toh)

	#Uploading waypoints and mission
	set_waypoint(start_index=0, waypoints=W)

	#Subscriber function to control mission parameters
	rospy.Subscriber(drone_ID+'/mavros/mission/waypoints', WaypointList, control_mission_cb, (set_cmd,set_cur_waypoint))

	#set current waypoint to gcs_cmd_waypoint
	set_cur_waypoint(wp_seq=gcs_cmd_wp)

	#time.sleep(1)

	#For doing mission
	if armed and cur_alt>=0.95*toh:
		set_mode(custom_mode='AUTO')
		while(bat>=critical):
			continue
	else:
		print(drone_ID + ' cannont continue on mission. Please check')
		exit()	


#Function called by drone_sub()
def hower_and_wait(drone_ID,set_mode):
	global armed, cur_alt, gcs_cmd_x_diff, gcs_cmd_y_diff, gcs_cmd_x, gcs_cmd_y, gcs_cmd_height, gcs_cmd_yaw
	
	set_mode(custom_mode='GUIDED')

	
	#Publisher to increase drone's altitude
	alt_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	inc_alt = PoseStamped()
	inc_alt.header.stamp = rospy.Time.now()
	inc_alt.pose.position.x = gcs_cmd_x + gcs_cmd_x_diff
	inc_alt.pose.position.y = gcs_cmd_y + gcs_cmd_y_diff
	inc_alt.pose.position.z = gcs_cmd_height
	[inc_alt.pose.orientation.x, inc_alt.pose.orientation.y, inc_alt.pose.orientation.z, inc_alt.pose.orientation.w] = quaternion_from_euler(0,0,gcs_cmd_yaw)
	
	'''
	pos_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
	init_pos = PositionTarget()
	init_pos.header.stamp = rospy.Time.now()
	init_pos.coordinate_frame = 1
	init_pos.type_mask = 0b001111000000
	init_pos.position.x = gcs_cmd_x + gcs_cmd_x_diff
	init_pos.position.y = gcs_cmd_y + gcs_cmd_y_diff
	init_pos.position.z = gcs_cmd_height
	init_pos.velocity.x = 0.5
	init_pos.velocity.y = 0.5
	init_pos.velocity.z = 0.3
	init_pos.yaw = gcs_cmd_yaw
	init_pos.yaw_rate = 20
	'''

	#Increase altitude and hover
	if armed:
		alt_pub.publish(inc_alt)
		while(cur_alt <= 0.95*gcs_cmd_height):
			time.sleep(1)
	else:
		print(droneID + ' not armed. Cannot initiate drone substitution')
		exit()


#Function called by drone_sub()
def go_to_home(drone_ID,set_mode,set_land,toh):
	global armed, cur_yaw, gcs_cmd_x_diff, gcs_cmd_y_diff

	#setting up mavros services
	mavros.set_namespace(drone_ID+'/mavros')
	
	
	#Publisher function to go to the home location
	home_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_position/local', PoseStamped, queue_size=1)
	land_pos = PoseStamped()
	land_pos.header.stamp = rospy.Time.now()
	land_pos.pose.position.x = 0 + gcs_cmd_x_diff
	land_pos.pose.position.y = 0 + gcs_cmd_y_diff
	land_pos.pose.position.z = toh
	[land_pos.pose.orientation.x, land_pos.pose.orientation.y, land_pos.pose.orientation.z, land_pos.pose.orientation.w] = quaternion_from_euler(0,0,cur_yaw)
	
	'''
	pos_pub = rospy.Publisher(drone_ID+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
	init_pos = PositionTarget()
	init_pos.header.stamp = rospy.Time.now()
	init_pos.coordinate_frame = 1
	init_pos.type_mask = 0b001111000000
	init_pos.position.x = 0 + gcs_cmd_x_diff
	init_pos.position.y = 0 + gcs_cmd_y_diff
	init_pos.position.z = toh
	init_pos.velocity.x = 0.5
	init_pos.velocity.y = 0.5
	init_pos.velocity.z = 1
	init_pos.yaw = cur_yaw
	init_pos.yaw_rate = 20
	'''

	if armed:

		#Preparing for landing
		set_mode(custom_mode='GUIDED')

		# go to home and hover
		home_pub.publish(land_pos)

		time.sleep(17)
		
		#Land
		set_land(0, 0, None, None, 0)
		while armed:
			time.sleep(1)

	else:
		print(drone_ID + ' not armed')
		exit()


#Function to subscribe to the GCS command and take action accordingly (runs on sepatate thread)
def drone_sub(drone_ID,toh,x_meters,y_meters,safe,low,critical):
	global status, gcs_cmd_str, armed, cur_alt

	# Definitions for rospy services
	set_mode = rospy.ServiceProxy(drone_ID+'/mavros/set_mode', SetMode)
	set_takeoff = rospy.ServiceProxy(drone_ID+'/mavros/cmd/takeoff', CommandTOL)
	set_land = rospy.ServiceProxy(drone_ID+'/mavros/cmd/land', CommandTOL)
	set_cur_waypoint = rospy.ServiceProxy(drone_ID+'/mavros/mission/set_current', WaypointSetCurrent)
	set_cmd = rospy.ServiceProxy(drone_ID+'/mavros/cmd/command', CommandLong)
	waypoints_clean = rospy.ServiceProxy(drone_ID+'/mavros/mission/clear', WaypointClear)
	set_waypoint = rospy.ServiceProxy(drone_ID+'/mavros/mission/push', WaypointPush)


	#Subscriber function to get GCS commands
	rospy.Subscriber('gcs_command', gcs_msg, gcs_command_cb)

	#Publisher function to publish status of drone
	pub_status = rospy.Publisher(drone_ID+'_status', String, queue_size=1)

	print(drone_ID + ' Ready')
	
	while not rospy.is_shutdown():

		if gcs_cmd_str == drone_ID + ' takeoff': 			#GCS takeoff and go to position command for drone			
			print(drone_ID + ' copy takeoff')
			pub_status.publish(drone_ID + ' takeoff')
			go_to_location(drone_ID,set_mode,set_takeoff,set_land,toh,x_meters,y_meters,safe)
			pub_status.publish(drone_ID + ' at location')

		if gcs_cmd_str == drone_ID + ' do mission':			#GCS normal mission command for drone
			print(drone_ID + ' copy doing mission')
			pub_status.publish(drone_ID + ' doing mission')
			normal_mission(drone_ID,waypoints_clean,set_waypoint,set_cur_waypoint,set_cmd,set_mode,x_meters,y_meters,toh,low,critical)
			pub_status.publish(drone_ID + ' need replacement')
		
		if gcs_cmd_str == drone_ID + ' replacement ready':			#GCS command for drone to suggest that another drone is available for replacement
			print(drone_ID + ' copy preparing for replacement')
			pub_status.publish(drone_ID + ' copy preparing for replacement')
			hower_and_wait(drone_ID,set_mode)
			pub_status.publish(drone_ID + ' ready for replacement')
		
		if gcs_cmd_str == drone_ID + ' land':					#GCS go to home location and land command for drone
			print(drone_ID + ' copy land')
			pub_status.publish(drone_ID + ' copy landing')
			go_to_home(drone_ID,set_mode,set_land,toh)
			pub_status.publish(drone_ID + ' grounded')
		
		if gcs_cmd_str == 'replacement for ' + drone_ID + ' not ready':	#GCS command for drone to suggest that no other drone is available for replacement
			print(drone_ID + ' copy error')
			set_land(0, 0, None, None, 0)
			while armed:
				time.sleep(1)
			exit()
		
		rospy.sleep(0.1)


#Function to simulate and publish charging and discharging of battery level (runs on sepatate thread)
def drone_bat(drone_ID):
	global bat, status, armed

	#Subscriber function to check if drone is armed or not 
	rospy.Subscriber(drone_ID+'/mavros/state', State, armed_cb)

	#pub_status = rospy.Publisher(drone_ID+'_status', String, queue_size=1)
	pub_bat = rospy.Publisher(drone_ID+'_battery', Int16, queue_size=1)
	
	while not rospy.is_shutdown():
		
		if armed:
			if bat>0:
				bat = bat - 5
				time.sleep(7)
			else:
				bat = 0
		if not armed:
			if bat<100:
				bat = bat + 100
			else:
				bat = 100

		pub_bat.publish(bat)
		#print(drone_ID + ' battery percent = %d' %(bat))
		#print(drone_ID + ' status = %s' %(status))
		rospy.sleep(0.1)



if __name__ == '__main__':

	#parser so that arguments can be ammended during run time
	parser = argparse.ArgumentParser(description='params')
	parser.add_argument('--dr_no', default=0, type=int)
	parser.add_argument('--toh', default=3, type=int)					#Desired Takeoff height for mission
	parser.add_argument('--surveillance_dis_x', default=5, type=int)	#Distance to cover from home location, in both North and South direction (in meters)
	parser.add_argument('--surveillance_dis_y', default=5, type=int)	#Distance to cover from home location, in both East and West direction (in meters)
	parser.add_argument('--safe_bat',default=90, type=int)				#Minimum safe battery level to authorize takeoff (percentage)
	parser.add_argument('--low_bat', default=70, type=int)				#Low battery level (percentage)
	parser.add_argument('--critical_bat', default=50, type=int)			#Critical battery level (percentage)

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
	if args.safe_bat-args.low_bat<20:
		print('Make sure low battery level is set to at least 20% below safe battery level')
		exit()

	#Making sure that critical battery level is sufficiently below low battery level
	if args.low_bat-args.critical_bat<20:
		print('Make sure critical battery level is set to at least 20% below low battery level')
		exit()

	drone_num = args.dr_no					
	toh = args.toh								
	x_meters = args.surveillance_dis_x		
	y_meters = args.surveillance_dis_y		
	safe = args.safe_bat						
	low = args.low_bat 							
	critical = args.critical_bat				

	#setting minimum takeoff height (min_toh = 8 meters) for safety
	#if toh<8:
		#toh=8

	rospy.init_node('drone_'+str(drone_num))

	#ID of drone (Ex. drone1 , drone2 , drone3 ...) 
	drone_ID = 'drone' + str(drone_num)

	thread1 = threading.Thread(target=drone_sub,args=(drone_ID,toh,x_meters,y_meters,safe,low,critical)) 
	thread2 = threading.Thread(target=drone_bat,args=(drone_ID,))

	thread1.start()
	thread2.daemon=True
	thread2.start()
