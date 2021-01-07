#!/usr/bin/env python3

# Code to control drone1
# running this code should cause drone1 to take commands from GCS and act accordingly
# Usage - python3 drone1_mission.py


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import rospy, mavros, time
import threading
from mavros import command

from std_msgs.msg import String, Int16
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

# battery percentage
bat = 100
# drone flight status
status = 'grounded'
# drone armed status
armed = False
# Command from GSC
cmd = 'none'


def armed_cb(data):
	global armed
	armed = data.armed


def command_cb(data):
	global cmd
	cmd = data.data


def drone_pub():
	global bat, status

	pub_status = rospy.Publisher('drone1_status', String, queue_size=1)
	pub_bat = rospy.Publisher('drone1_battery', Int16, queue_size=1)
	
	while not rospy.is_shutdown():
		pub_status.publish(status)
		pub_bat.publish(bat)
		rospy.sleep(0.1)
 

def drone_sub():
	global status, cmd, armed

	#setting up mavros
	mavros.set_namespace('drone1/mavros')
	set_mode = rospy.ServiceProxy('/drone1/mavros/set_mode', SetMode)
	set_takeoff = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)
	set_landing = rospy.ServiceProxy('/drone1/mavros/cmd/land', CommandTOL)

	rospy.Subscriber('gcs_command', String, command_cb)
	
	while not rospy.is_shutdown():
		if cmd == 'drone1_takeoff':
			print('drone1 copy takeoff')
			status = 'flying'
			set_mode(custom_mode='GUIDED')
			mavros.command.arming(True)
			time.sleep(1)
			set_takeoff(0, 0, None, None, 5)
		if cmd == 'drone1_land':
			print('drone1 copy land')
			status = 'grounded'
			set_landing(0, 0, None, None, 0)
			while armed:
				time.sleep(1)
		rospy.sleep(0.1)


def drone_bat():
	global bat, status, armed
	counter = 0

	rospy.Subscriber('drone1/mavros/state', State, armed_cb)
	
	while not rospy.is_shutdown():
		if armed and counter%10==0:
			if bat>0:
				bat = bat - 5
			else:
				bat = 0
			print('drone1 battery percent = %d' %(bat))
			print('drone1 status = %s' %(status))
		if not armed and counter%5==0:
			if bat<100:
				bat = bat + 5
			else:
				bat = 100
			print('drone1 battery percent = %d' %(bat))
			print('drone1 status = %s' %(status))
		counter = counter + 1
		rospy.sleep(0.1)


if __name__ == '__main__':
	rospy.init_node('drone_1')

	thread1 = threading.Thread(target=drone_pub,args=())
	thread2 = threading.Thread(target=drone_sub,args=())
	thread3 = threading.Thread(target=drone_bat,args=())

	thread1.start()
	thread2.start()
	thread3.start()
