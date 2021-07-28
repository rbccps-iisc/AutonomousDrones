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


def main():
	

	old_settings = termios.tcgetattr(sys.stdin)

	print('Ready')
	
	while(True):

		try:

			tty.setcbreak(sys.stdin.fileno())

			if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):

				print('in')

				event = sys.stdin.read(1)
				event = string.lower(event)

				print(event)

				if event == 'w':	
					print("Forward")
				elif event == 's':	
					print("Back")
				elif event == 'a':
					print("Left")
				elif event == 'd':	
					print("Right")
				else:
					print("Invalid command")
					break

		finally:
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
		
if __name__ == '__main__':
	main()