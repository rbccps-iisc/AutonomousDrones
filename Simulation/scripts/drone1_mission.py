#!/usr/bin/env python3

# Code to control drone1
# running this code should cause drone1 to take commands from GCS and act accordingly
# Usage - python3 drone1_mission.py


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import argparse
import threading
import time, math

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header

import rospy, mavros
from mavros import command

from std_msgs.msg import String, Int16
#from mavros_msgs.msg import State
#from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped

from multidrone_mission.msg import drone


# battery percentage
bat = 100
# drone flight status
status = 'drone1 grounded'
# drone armed status
armed = False

# Command from GSC
gcs_cmd_str = 'none'
gcs_cmd_lat = float('nan')
gcs_cmd_lon = float('nan')
gcs_cmd_height = float('nan')
gcs_cmd_yaw = float('nan')
gcs_cmd_wp = 1

#Current reletive altitude
cur_alt = 0


#Function callback for 'drone1/mavros/state'
def armed_cb(data):
	global armed
	armed = data.armed

#Function callback for 'drone1/mavros/local_position/pose'
def alt_cb(data):
	global cur_alt
	cur_alt = data.pose.position.z

#Function callback for 'gcs_command'
def gcs_command_cb(data):
	global gcs_cmd_str, gcs_cmd_lat, gcs_cmd_lon, gcs_cmd_height, gcs_cmd_yaw, gcs_cmd_wp
	gcs_cmd_str = data.command
	gcs_cmd_lat = data.lat
	gcs_cmd_lon = data.lon
	gcs_cmd_height = data.height
	gcs_cmd_yaw = data.yaw 	
	gcs_cmd_wp = data.waypoint


#Function callback for '/drone1/mavros/mission/waypoints'
def control_mission_cb(data):
	cur_wp = data.current_seq
	
	#Service to set desired waypoint 
	set_cur_waypoint = rospy.ServiceProxy('/drone1/mavros/mission/set_current', WaypointSetCurrent)

	#service to set mission parameters
	set_cmd = rospy.ServiceProxy('/drone1/mavros/cmd/command', CommandLong)

	#command = 178 (for setting ground speed) ; param2 = speed (m/s)
	set_cmd(command=178, param2=1)

	#command = 115 (for setting yaw) ; param1 = yaw angle (deg) ; param2 = yaw speed (deg/s) ; param4 = 0 (global frame)
	if cur_wp==2:
		set_cmd(command=115, param1=0, param2=45, param4=0)		# W[1] to W[2] (Refer to below diageam) (param1=0 -> Facing North)
	if cur_wp==3:
		set_cmd(command=115, param1=90, param2=45, param4=0)	# W[2] to W[3] (Refer to below diageam) (param1=90 -> Facing East)
	if cur_wp==4:
		set_cmd(command=115, param1=180, param2=45, param4=0)	# W[3] to W[4] (Refer to below diageam) (param1=180 -> Facing South)
	if cur_wp==5:
		set_cur_waypoint(wp_seq=1)								# W[5]==W[1] (Refer to below diageam) (Required to keep mission continued)
		set_cmd(command=115, param1=270, param2=45, param4=0)	# W[4] to W[5]/W[1] (Refer to below diageam) (param1=270 -> Facing West)


#Function called by normal_mission()
def waypoint_dataset(home_lat,home_lon,dLat,dLon,toh):

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
	wp.x_lat = home_lat				#Latitude
	wp.y_long = home_lon			#Longitude
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
	wp.x_lat = home_lat + dLat		#Latitude
	wp.y_long = home_lon - dLon		#Longitude
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
	wp.x_lat = home_lat + dLat		#Latitude
	wp.y_long = home_lon + dLon		#Longitude
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
	wp.x_lat = home_lat - dLat		#Latitude
	wp.y_long =  home_lon + dLon		#Longitude
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
	wp.x_lat = home_lat - dLat		#Latitude
	wp.y_long = home_lon - dLon		#Longitude
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
	wp.x_lat = home_lat + dLat		#Latitude
	wp.y_long = home_lon - dLon		#Longitude
	wp.z_alt = toh					#altitude
	W.append(wp)
	
	#print(W)
	return(W)


#Function called by normal_mission() and go_to_location()
def dis_to_gps(dLat_meters,dLon_meters):

	earth_radius=6378137.0 #Radius of "spherical" earth
	#Converting desired trajectories of drone in meters to GPS coordinates (in radians)
	dLat = dLat_meters/earth_radius
	dLon = dLon_meters/(earth_radius*math.cos(math.pi*home_lat/180))
	#Converting from radians to degrees
	dLat = dLat * 180/math.pi
	dLon = dLon * 180/math.pi

	return (dLat, dLon)


#Function called by drone_sub()
def go_to_location(home_lat,home_lon,home_alt,toh,dLat_meters,dLon_meters,safe):
	global bat, armed, cur_alt, gcs_cmd_lat, gcs_cmd_lon, gcs_cmd_height, gcs_cmd_yaw

	#Function to convert distance (in meters) to latitude and longitude
	dLat, dLon = dis_to_gps(dLat_meters,dLon_meters)

	#Setup for initial sorty
	if math.isnan(gcs_cmd_lat):
		gcs_cmd_lat = home_lat + dLat
	if math.isnan(gcs_cmd_lon):
		gcs_cmd_lon = home_lon - dLon
	if math.isnan(gcs_cmd_height):
		gcs_cmd_height = toh
	if math.isnan(gcs_cmd_yaw):
		gcs_cmd_yaw = math.pi/2

	#setting up mavros services
	mavros.set_namespace('drone1/mavros')
	set_mode = rospy.ServiceProxy('/drone1/mavros/set_mode', SetMode)
	set_takeoff = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)
	set_land = rospy.ServiceProxy('/drone1/mavros/cmd/land', CommandTOL)
	
	#Subscriber function to get current altitude of drone
	rospy.Subscriber('drone1/mavros/local_position/pose', PoseStamped, alt_cb)

	#Publisher function to go to the desired initial location
	pos_pub = rospy.Publisher('/drone1/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
	init_pos = GeoPoseStamped()
	init_pos.header.stamp = rospy.Time.now()
	init_pos.pose.position.latitude = gcs_cmd_lat
	init_pos.pose.position.longitude = gcs_cmd_lon
	init_pos.pose.position.altitude = home_alt + gcs_cmd_height
	[init_pos.pose.orientation.x, init_pos.pose.orientation.y, init_pos.pose.orientation.z, init_pos.pose.orientation.w] = quaternion_from_euler(0,0,gcs_cmd_yaw)

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

		#if battery level not sufficient
		else:
			print('drone1 battery level below safe level. Therefore cannot arm and takeoff')
			exit()
	
	#if drone already armed 
	else:
		print('drone1 already armed. Hence cannont takeoff')
		set_land(0, 0, None, None, 0)
		while armed:
			time.sleep(1)
		exit()
		

#Function called by drone_sub()
def normal_mission(home_lat,home_lon,home_alt,toh,dLat_meters,dLon_meters,low,critical):
	global bat, armed, gcs_cmd_wp, cut_alt

	#Function to convert distance (in meters) to latitude and longitude
	dLat, dLon = dis_to_gps(dLat_meters,dLon_meters)

	#Clear any existing mission on drone1 (if any)
	waypoints_clean = rospy.ServiceProxy('drone1/mavros/mission/clear', WaypointClear)
	waypoints_clean.call()

	#Create waypoints for function by calling waypoint_dataset() [defined above]
	W = waypoint_dataset(home_lat,home_lon,dLat,dLon,toh)

	#setting up mavros services
	set_waypoint = rospy.ServiceProxy('/drone1/mavros/mission/push', WaypointPush)
	set_cur_waypoint = rospy.ServiceProxy('/drone1/mavros/mission/set_current', WaypointSetCurrent)
	set_mode = rospy.ServiceProxy('/drone1/mavros/set_mode', SetMode)

	#Uploading waypoints and mission
	set_waypoint(start_index=0, waypoints=W)

	#set current waypoint to 1st point
	set_cur_waypoint(wp_seq=gcs_cmd_wp)

	#Subscriber function to control mission parameters
	rospy.Subscriber('/drone1/mavros/mission/waypoints', WaypointList, control_mission_cb)

	#For doing mission
	if armed and cur_alt>=0.95*toh:
		set_mode(custom_mode='AUTO')
		while(bat>=critical):
			continue
	else:
		print('drone1 cannont continue on mission. Please check')
		exit()	


#Function called by drone_sub()
def hower_and_wait(toh, home_alt):
	global armed, cur_alt, gcs_cmd_lat, gcs_cmd_lon, gcs_cmd_height, gcs_cmd_yaw
	
	#Service to stop mission (auto mode) and allow drone to accept commands
	set_mode = rospy.ServiceProxy('drone1/mavros/set_mode', SetMode)
	set_mode(custom_mode='GUIDED')

	#Publisher to increase drone's altitude
	alt_pub = rospy.Publisher('/drone1/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
	inc_alt = GeoPoseStamped()
	inc_alt.header.stamp = rospy.Time.now()
	inc_alt.pose.position.latitude = gcs_cmd_lat
	inc_alt.pose.position.longitude = gcs_cmd_lon
	inc_alt.pose.position.altitude = home_alt + gcs_cmd_height
	[inc_alt.pose.orientation.x, inc_alt.pose.orientation.y, inc_alt.pose.orientation.z, inc_alt.pose.orientation.w] = quaternion_from_euler(0,0,gcs_cmd_yaw)

	#Increase altitude and hover
	if armed:
		alt_pub.publish(inc_alt)
		while(cur_alt <= 0.95*gcs_cmd_height):
			time.sleep(1)
	else:
		print('drone1 not armed. Cannot initiate drone substitution')
		exit()


#Function called by drone_sub()
def go_to_home(home_lat,home_lon,home_alt,toh):
	global armed

	#setting up mavros services
	mavros.set_namespace('drone1/mavros')
	set_mode = rospy.ServiceProxy('/drone1/mavros/set_mode', SetMode)
	set_land = rospy.ServiceProxy('/drone1/mavros/cmd/land', CommandTOL)

	#Publisher function to go to the home location
	home_pub = rospy.Publisher('/drone1/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)
	land_pos = GeoPoseStamped()
	land_pos.header.stamp = rospy.Time.now()
	land_pos.pose.position.latitude = home_lat
	land_pos.pose.position.longitude = home_lon
	land_pos.pose.position.altitude = home_alt + toh
	[land_pos.pose.orientation.x, land_pos.pose.orientation.y, land_pos.pose.orientation.z, land_pos.pose.orientation.w] = quaternion_from_euler(0,0,0)

	if armed:

		#Preparing for landing
		set_mode(custom_mode='GUIDED')

		# go to home and hover
		home_pub.publish(land_pos)

		time.sleep(20)
		
		#Land
		set_land(0, 0, None, None, 0)
		while armed:
			time.sleep(1)

	else:
		print('drone1 not armed')
		exit()


#Function to subscribe to the GCS command and take action accordingly (runs on sepatate thread)
def drone_sub(home_lat,home_lon,home_alt,toh,dLat_meters,dLon_meters,safe,low,critical):
	global status, gcs_cmd_str, armed, cur_alt

	#setting up mavros services
	mavros.set_namespace('drone1/mavros')
	set_land = rospy.ServiceProxy('/drone1/mavros/cmd/land', CommandTOL)

	#Subscriber function to get GCS commands
	rospy.Subscriber('gcs_command', drone, gcs_command_cb)

	#Publisher function to publish status of drone
	pub_status = rospy.Publisher('drone1_status', String, queue_size=1)
	
	while not rospy.is_shutdown():

		if gcs_cmd_str == 'drone1 takeoff': 			#GCS takeoff and go to position command for drone1			
			print('drone1 copy takeoff')
			pub_status.publish('drone1 takeoff')
			go_to_location(home_lat,home_lon,home_alt,toh,dLat_meters,dLon_meters,safe)
			pub_status.publish('drone1 at location')
			print('drone1 at location')

		if gcs_cmd_str == 'drone1 do mission':			#GCS normal mission command for drone1
			print('drone1 copy doing mission')
			pub_status.publish('drone1 doing mission')
			normal_mission(home_lat,home_lon,home_alt,toh,dLat_meters,dLon_meters,low,critical)
			pub_status.publish('drone1 need replacement')
		
		if gcs_cmd_str == 'drone1 replacement ready':			#GCS command for drone1 to suggest that another drone is available for replacement
			print('drone1 copy preparing for replacement')
			pub_status.publish('drone1 copy preparing for replacement')
			hower_and_wait(toh,home_alt)
			pub_status.publish('drone1 ready for replacement')
		
		if gcs_cmd_str == 'drone1 land':					#GCS go to home location and land command for drone1
			print('drone1 copy land')
			pub_status.publish('drone1 copy landing')
			go_to_home(home_lat,home_lon,home_alt,toh)
			pub_status.publish('drone1 grounded')
		
		if gcs_cmd_str == 'replacement for drone1 not ready':	#GCS command for drone1 to suggest that no other drone is available for replacement
			print('drone1 copy error')
			set_land(0, 0, None, None, 0)
			while armed:
				time.sleep(1)
			exit()
		
		rospy.sleep(0.1)


#Function to simulate and publish charging and discharging of battery level (runs on sepatate thread)
def drone_bat():
	global bat, status, armed
	counter = 0

	#Subscriber function to check if drone is armed or not 
	rospy.Subscriber('drone1/mavros/state', State, armed_cb)

	#pub_status = rospy.Publisher('drone1_status', String, queue_size=1)
	pub_bat = rospy.Publisher('drone1_battery', Int16, queue_size=1)
	
	while not rospy.is_shutdown():
		
		if armed and counter%50==0:
			if bat>0:
				bat = bat - 5
			else:
				bat = 0
		if not armed and counter%1==0:
			if bat<100:
				bat = bat + 5
			else:
				bat = 100

		pub_bat.publish(bat)
		#print('drone1 battery percent = %d' %(bat))
		#print('drone1 status = %s' %(status))
		counter = counter + 1
		rospy.sleep(0.1)



if __name__ == '__main__':

	#parser so that arguments can be ammended during run time
	parser = argparse.ArgumentParser(description='params')
	parser.add_argument('--check', default=False, type=bool)
	parser.add_argument('--home_lat', default=13.0272048)				#GPS Latitude coordinate of home location in Current Airfield
	parser.add_argument('--home_lon', default=77.563607)				#GPS Longitude coordinate of home location in Current Airfield
	parser.add_argument('--home_alt', default=915.0)					#Altitude of home location in Current Airfield
	parser.add_argument('--toh', default=6, type=int)					#Desired Takeoff height for mission
	parser.add_argument('--surveillance_dis_x', default=6, type=int)	#Distance to cover from home location, in both North and South direction (in meters)
	parser.add_argument('--surveillance_dis_y', default=6, type=int)	#Distance to cover from home location, in both East and West direction (in meters)
	parser.add_argument('--safe_bat',default=80, type=int)				#Minimum safe battery level to authorize takeoff (percentage)
	parser.add_argument('--low_bat', default=60, type=int)				#Low battery level (percentage)
	parser.add_argument('--critical_bat', default=40, type=int)			#Critical battery level (percentage)

	args = parser.parse_args()

	#Making sure that these params are checked before operation
	if not args.check:
		print('Please check parser \'params\' before proceeding')
		print('If parser arguments are checked and correct, proceed as - python3 drone1_mission.py --check=True')
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
	#if args.safe_bat-args.low_bat<20:
	#	print('Make sure low battery level is set to at least 20% below safe battery level')
	#	exit()

	#Making sure that critical battery level is sufficiently below low battery level
	#if args.low_bat-args.critical_bat<20:
	#	print('Make sure critical battery level is set to at least 20% below low battery level')
	#	exit()

	home_lat = args.home_lat					
	home_lon = args.home_lon					
	home_alt = args.home_alt 					
	toh = args.toh								
	dLat_meters = args.surveillance_dis_x		
	dLon_meters = args.surveillance_dis_y		
	safe = args.safe_bat						
	low = args.low_bat 							
	critical = args.critical_bat				

	#setting minimum takeoff height (min_toh = 8 meters) for safety
	if toh<8:
		toh=8

	rospy.init_node('drone_1')

	thread1 = threading.Thread(target=drone_sub,args=(home_lat,home_lon,home_alt,toh,dLat_meters,dLon_meters,safe,low,critical)) 
	thread2 = threading.Thread(target=drone_bat,args=())

	thread1.start()
	thread2.daemon=True
	thread2.start()
