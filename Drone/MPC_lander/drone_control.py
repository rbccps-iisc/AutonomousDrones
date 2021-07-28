#!/usr/bin/env python2

from __future__ import print_function
from os import sys, path
import select, termios, tty, rospy, argparse, mavros, threading, time, readline, signal, select, tf, quadprog, math, tf2_ros, tf2_geometry_msgs, csv

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from datetime import datetime
from opencv.lib_aruco_pose import ArucoSingleTracker
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Header, Float32, Float64, Empty, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from rosgraph_msgs.msg import Clock

from mavros import command
from mavros_msgs.msg import PositionTarget, Altitude, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from nav_msgs.msg import Path, Odometry
from geographic_msgs.msg import GeoPoseStamped
from visualization_msgs.msg import Marker
from math import pow, sqrt

#from gazebo_msgs.msg import ModelStates, ContactsState

from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import numpy as np
from MPC import MPC_solver

class LandUsingMPC():

	def __init__(self,
				drone_ID 	= '',
				aruco_lat 	= 13.0272068,
				aruco_lon 	= 77.5636419,
				home_alt 	= 0.0,
				call 		= False, 
				first_land	= True
				):

		#Public class variables 
		self.drone_ID 					= drone_ID
		self.aruco_lat 					= aruco_lat
		self.aruco_lon 					= aruco_lon
		self.home_alt 					= home_alt
		self.call 						= call
		self.first_land					= first_land

		#Private class variables
		self.__hz                       = 10.0
		self.__n                        = 15
		self.__t 						= 1/self.__hz
		self.__cont                     = 0
		self.__home_xy_recorded  		= False
		self.__home_z_recorded 			= False
		self.__cart_x					= 0.0
		self.__cart_y 		           	= 0.0
		self.__cart_z					= 0.0
		self.__vel_x 					= 0.0
		self.__vel_y 					= 0.0
		self.__vel_z 					= 0.0
		self.__delta_time				= 0.0
		self.__home_x 					= 0.0
		self.__home_y 					= 0.0
		self.__home_z 					= 0.0
		self.__ekf_x 					= 0
		self.__ekf_y 					= 0
		self.__ekf_z 					= 0
		self.__desired_x 				= 0.0
		self.__desired_y 				= 0.0
		self.__desired_z 				= 0.0
		self.__limit_x 					= 220
		self.__limit_y 					= 220
		self.__limit_z 					= 220
		self.__roll 				    = 0.0
		self.__pitch 				    = 0.0
		self.__yaw 				        = 0.0
		self.__home_yaw                 = 0
		self.__br                       = tf.TransformBroadcaster()
		self.__discard_samples          = 20	                    #samples to discard before gps normalizes
		self.__alt                      = 0.0
		self.__start_y                  = 0.0
		self.__cached_var_x             = {}
		self.__cached_var_y             = {}
		self.__cached_var_z             = {}
		self.__acc 						= 0
		self.__max_acc                  = 0
		self.__detected_aruco           = False
		self.__aruco_x 					= 0
		self.__aruco_y 					= 0
		self.__aruco_z 					= 0
		self.__prev_vel                 = 0
		self.__prev_time                = rospy.Time()
		self.__start_clock              = False
		self.__is_reached               = False
		self.__first_attempt			= True
		self.__full_avg                 = 0.0
		self.__coord                    = [0,0,0]
		self.__visible                  = False


	def __clock_cb(self, data):
		self.__current_time = data.clock.secs

	def __imu_cb(self, data):
		orientation_q = data.orientation
		orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
		(self.__roll, self.__pitch, self.__yaw) = euler_from_quaternion(orientation_list)
		(self.__roll, self.__pitch, self.__yaw) = (self.__roll * 180.0/3.1416, self.__pitch * 180.0/3.1416, self.__yaw  * 180.0/3.1416)
		self.__acc = data.linear_acceleration.x
		self.__br.sendTransform((0, 0, 0), (-orientation_q.x, -orientation_q.y, -orientation_q.z, orientation_q.w), rospy.Time.now(), "base_link_att_comp", "base_link")

	def __gps_local_cb(self, data):
		self.__cart_x = data.pose.pose.position.x
		self.__cart_y = data.pose.pose.position.y
		self.__cart_z = data.pose.pose.position.z
		if self.__home_xy_recorded is False and self.__cart_x != 0 and self.__cart_y != 0:
			self.__home_x = self.__cart_x
			self.__home_y = self.__cart_y
			self.__discard_samples = self.__discard_samples - 1
			if(self.__discard_samples <= 0):
				self.__desired_x = self.__cart_x                          #to set home position as initial desired position
				self.__desired_y = self.__cart_y
				self.__start_y = self.__home_y
				self.__home_xy_recorded = True

	def __pose_cb(self, data):
		position = data.pose.position
		self.__ekf_x = position.x
		self.__ekf_y = position.y
		self.__ekf_z = position.z

	def __alt_cb(self, data):
		self.__alt = data.amsl
		if self.home_alt==0.0:
			self.home_alt=data.amsl

	def __velocity_cb(self, data):
		self.__vel_x = data.twist.linear.x
		self.__vel_y = data.twist.linear.y
		self.__vel_z = data.twist.linear.z
		curr_time = data.header.stamp
		self.__delta_time = (curr_time - self.__prev_time).to_sec()
		self.__prev_time = curr_time

	def __armed_cb(self, data):
		self.__armed = data.armed

	def __aruco_coord_cb(self, data):
		self.__coord[0] = data.x
		self.__coord[1] = data.y
		self.__coord[2] = data.z

	def __aruco_visible_cb(self, data):
		self.__visible = data.data

	def __range_cb(self, data):
		self.__cart_z = data.range


	#Velocity Publisher Object
	def __twist_obj(self, x, y, z, a, b, c):
		move_cmd = TwistStamped(header=Header(stamp=rospy.get_rostime()))
		move_cmd.twist.linear.x = x
		move_cmd.twist.linear.y = y
		move_cmd.twist.linear.z = z
		move_cmd.twist.angular.x = a
		move_cmd.twist.angular.y = b
		move_cmd.twist.angular.z = c
		return move_cmd

	#Position Publisher Object
	def __geo_pose_obj(self, x, y, z, a, b, c, d):
		pose_cmd = GeoPoseStamped(header=Header(stamp=rospy.get_rostime()))
		pose_cmd.pose.position.latitude = x
		pose_cmd.pose.position.longitude = y
		pose_cmd.pose.position.altitude = z
		pose_cmd.pose.orientation.x = a
		pose_cmd.pose.orientation.y = b
		pose_cmd.pose.orientation.z = c
		pose_cmd.pose.orientation.w = d
		return pose_cmd


	def land(self):

		self.__file_str = path.dirname(path.abspath(__file__)) +'/_' + str(datetime.now().month) + '_' + str(datetime.now().day) + '_' + str(datetime.now().hour) + '_' + str(datetime.now().minute) + '.csv'
		if not path.isfile(self.__file_str):
			csvfile = open(self.__file_str,'w')
			fieldnames = ['Time','cart_x','cart_y','cart_z','vel_x','vel_y','vel_z','desired_x','desired_y','desired_z','aruco_x','aruco_y']
			writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
			writer.writeheader()
			csvfile.close()
		csvfile = open(self.__file_str,'a')
		writer = csv.writer(csvfile)


		if not self.call:
			rospy.init_node('MAVROS_Listener')
		
		rate = rospy.Rate(self.__hz)

		tf_buff = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(tf_buff)


		#ROS Subscribers
		if self.first_land:
			rospy.Subscriber(self.drone_ID+"/mavros/imu/data", Imu, self.__imu_cb)
			rospy.Subscriber(self.drone_ID+"/mavros/altitude", Altitude, self.__alt_cb)
			rospy.Subscriber(self.drone_ID+"/mavros/global_position/local", Odometry, self.__gps_local_cb)
			rospy.Subscriber(self.drone_ID+"/mavros/local_position/pose", PoseStamped, self.__pose_cb)
			rospy.Subscriber(self.drone_ID+"/mavros/local_position/velocity_local", TwistStamped, self.__velocity_cb)
			rospy.Subscriber(self.drone_ID+"/mavros/state", State, self.__armed_cb)
			rospy.Subscriber("/aruco_coord", Vector3, self.__aruco_coord_cb)
			rospy.Subscriber("/aruco_visible", Bool, self.__aruco_visible_cb)
			rospy.Subscriber('clock', Clock, self.__clock_cb)
			#rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, range_cb)

		#ROS Publishers
		pub_vel = rospy.Publisher(self.drone_ID+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
		pub_goto = rospy.Publisher(self.drone_ID+'/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

		#ROS Services
		set_arming = rospy.ServiceProxy(self.drone_ID+'/mavros/cmd/arming', CommandBool)
		set_mode = rospy.ServiceProxy(self.drone_ID+'/mavros/set_mode', SetMode)
		set_takeoff = rospy.ServiceProxy(self.drone_ID+'/mavros/cmd/takeoff', CommandTOL)
		set_landing = rospy.ServiceProxy(self.drone_ID+'/mavros/cmd/land', CommandTOL)



		if not self.call:

			mavros.command.arming(True)

			#print(self.__cart_z)
			if(self.__cart_z < 1):
				set_mode(0, 'AUTO.TAKEOFF')

			while self.__cart_z < 9.5: continue


		if self.call:
			print("REACHED mpc")
		

		data_timer = 0.0
		hold_timer = 0.0
		hover_timer = 0.0


		while not rospy.is_shutdown():
			
			out_of_view_count = 0
			self.__cont = self.__cont + 1

			start_timer = time.time()

			x_cm,y_cm,z_cm = self.__coord

			if self.__home_z_recorded is False and self.__cart_z != 0 and self.__yaw != 0:
				self.__home_yaw = self.__yaw
				self.__home_z = self.__cart_z
				self.__home_z_recorded = True

			aruco_cam_pos = tf2_geometry_msgs.PointStamped(header=Header(stamp=rospy.Time.now(), frame_id='base_link'))
			aruco_cam_pos.point.x = y_cm/100
			aruco_cam_pos.point.y = x_cm/100
			aruco_cam_pos.point.z = z_cm/100

			try:
				p = tf_buff.transform(aruco_cam_pos, "base_link_att_comp", timeout=rospy.Duration(0.005))
			except :
				print('Oops')

			################################ MPC ###################################
			if(self.__visible and self.__is_reached):
				out_of_view_count = 0
				self.__detected_aruco = True
				print("----------------------------SEEN------------------------------------")
				self.__aruco_x = p.point.x
				self.__aruco_y = p.point.y
				self.__aruco_z = p.point.z
				velocity_x_des, self.__cached_var_x, diff = MPC_solver(self.__aruco_x, 0, self.__limit_x, 0, self.__n, self.__t, True, variables = self.__cached_var_x, vel_limit = 0.5, acc = 2, curr_vel = self.__vel_x)
				velocity_y_des, self.__cached_var_y, _ = MPC_solver(self.__aruco_y, 0, self.__limit_y, 0, self.__n, self.__t, True, variables = self.__cached_var_y, vel_limit = 0.5, acc = 2, curr_vel = self.__vel_y)
				velocity_z_des, self.__cached_var_z, _ = MPC_solver(self.__aruco_z, 0, self.__limit_z, 0, self.__n, self.__t, True, variables = self.__cached_var_z, vel_limit = 0.5, acc = 0, curr_vel = self.__vel_z, pos_cost=1, vel_cost=1000, debug=False)

				pub_vel.publish(self.__twist_obj(velocity_x_des, velocity_y_des, velocity_z_des, 0.0, 0.0, 0.0))

				self.__acc = diff / self.__t


			elif(not self.__is_reached):
				start_time = rospy.Time.now()
				print("----------------------------NOT SEEN------------------------------------")
				
				if self.__first_attempt:
					pub_goto.publish(self.__geo_pose_obj(self.aruco_lat, self.aruco_lon, self.home_alt+10, 0, 0, 0, 1))
				else:
					pub_goto.publish(self.__geo_pose_obj(self.aruco_lat, self.aruco_lon, self.__alt, 0, 0, 0, 1))

				if(abs(self.__vel_x) < 0.1 and abs(self.__vel_y) < 0.1):
					hover_timer = hover_timer + self.__delta_time

					print(hover_timer)

					if(hover_timer > 0.5):
						self.__is_reached = True
						failsafe_timer = time.time()

			
			else:
				print("Drifted")

				pub_vel.publish(self.__twist_obj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

				self.__first_attempt = False

				if out_of_view_count==5:
					self.__is_reached = False
					out_of_view_count = 0
				else:
					out_of_view_count = out_of_view_count + 1


			data_timer = data_timer + self.__delta_time
			hold_timer = hold_timer + self.__delta_time
			

			if(hold_timer < 0.2):
				#print("Hold timer:", hold_timer)
				set_mode(0, 'OFFBOARD')


			if(self.__visible == True and self.__cart_z <= 1.5):
				velocity_x_des = velocity_y_des = 0

				print("Time to land:\t", hold_timer)

				self.__is_reached = False
				self.__detected_aruco = False
				
				set_mode(0, 'AUTO.LAND')

				csvfile.close()

				if not self.call:
					sys.exit()
				else:
					while self.__armed: 
						continue
					return

			if(self.__is_reached and time.time()-failsafe_timer>20):

				print('MPC FAILSAFE ACTIVATED')
				
				set_mode(0, 'AUTO.LAND')

				csvfile.close()

				while self.__armed: 
					continue

				print('MPC FAILSAFE COMPLETED')

				if not self.call:
					sys.exit()
				else:
					return

			self.__full_avg = (self.__full_avg + time.time()-start_timer)

			rate.sleep()

		print("Full Average Time =\t", self.__full_avg/self.__cont)
		

	
if __name__ == "__main__":

	mavros.set_namespace("/mavros")
	np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
	
	land_obj = LandUsingMPC(drone_ID='', aruco_lat=13.0272068, aruco_lon=77.5636419, home_alt=0.0, call=False, first_land=True)
	land_obj.land()
