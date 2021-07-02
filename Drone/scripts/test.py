#!/usr/bin/env python

from os import sys, path

import argparse
import threading
import time, math
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Header

import rospy, mavros
from mavros import command

import sys
import select
import tty
import string
import termios

from std_msgs.msg import String, Int16, Bool

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry

cur_alt = 0.0

def alt_cb(data):
	global cur_alt
	cur_alt = data.pose.pose.position.z

def main():
	global cur_alt

	rospy.init_node('test_code')
	mavros.set_namespace('mavros')

	rospy.Subscriber('/mavros/global_position/local', Odometry, alt_cb)

	set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
	set_param = rospy.ServiceProxy('/mavros/param/set',ParamSet)
	pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)

	takeoff_alt = ParamValue()
	takeoff_alt.real = 3.0

	move_cmd = TwistStamped()
	move_cmd.twist.linear.x = 0
	move_cmd.twist.linear.y = 0
	move_cmd.twist.linear.z = 0
	move_cmd.twist.angular.x = 0
	move_cmd.twist.angular.y = 0
	move_cmd.twist.angular.z = 0

	mavros.command.arming(True)
				
	time.sleep(1)
	
	set_param(param_id='MIS_TAKEOFF_ALT', value=takeoff_alt) 
	set_mode(0, 'AUTO.TAKEOFF')

	while(cur_alt <= 2.7):
		time.sleep(1)

	old_settings = termios.tcgetattr(sys.stdin)

	print('Ready')

	'''
	pub_vel.publish(move_cmd)
	set_mode(0, 'OFFBOARD')
	pub_vel.publish(move_cmd)
	time.sleep(5)
	set_mode(0, 'AUTO.LAND')
	'''

	

	while not rospy.is_shutdown():

		try:

			tty.setcbreak(sys.stdin.fileno())

			if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):

				event = sys.stdin.read(1)
				event = string.lower(event)

				if event == 'w':	
					print("Forward")
					move_cmd.twist.linear.x = 1.0

				elif event == 's':	
					print("Back")
					move_cmd.twist.linear.x = -1.0

				elif event == 'a':
					print("Left")
					move_cmd.twist.linear.y = 1.0

				elif event == 'd':	
					print("Right")
					move_cmd.twist.linear.y = -1.0

				else:         
					set_mode(0, 'AUTO.LAND')
					break

				pub_vel.publish(move_cmd)
				set_mode(0, 'OFFBOARD')
				pub_vel.publish(move_cmd)
				time.sleep(1)
				move_cmd.twist.linear.x = 0.0
				move_cmd.twist.linear.y = 0.0
				pub_vel.publish(move_cmd)

		finally:
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
	
if __name__ == '__main__':
	main()