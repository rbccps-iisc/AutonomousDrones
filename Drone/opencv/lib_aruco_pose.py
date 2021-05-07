#!/usr/bin/env python2
"""
This demo calculates multiple things for different scenarios.

IF RUNNING ON A PI, BE SURE TO sudo modprobe bcm2835-v4l2

Here are the defined reference frames:

TAG:
				A y
				|
				|
				|tag center
				O---------> x

CAMERA:


				X--------> x
				| frame center
				|
				|
				V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
	> from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
	> position of the Camera in Tag axis: -R_ct.T*tvec
	> Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
	> Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
	> R_tf1 = R_cf2 an symmetric = R_f


"""
import rospy, os
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import cv2.aruco as aruco
from sensor_msgs.msg import Image
import sys, time, math
from datetime import datetime
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class ArucoSingleTracker():
	def __init__(self,
				id_to_find,
				marker_size,
				camera_matrix,
				camera_distortion,
				camera_size=[640,480],
				show_video=False,
				simulation=False,
				record_video=False,
				camera_topic='',
				drone_ID=''
				):
		
		if simulation:
			rospy.init_node(camera_topic.split('/')[1])
		else:
			rospy.init_node('cam_capture')

		rospy.Subscriber(camera_topic, Image, self.imageCb)

		rospy.Subscriber(drone_ID+'/run_aruco_detect', Bool, self.runCb)
		
		self.bridge = CvBridge()
	 
		self.frame = np.zeros((480,640,3), np.uint8)

		self.run = False

		self.id_to_find     = id_to_find
		self.marker_size    = marker_size
		self._show_video    = show_video
		self.simulation     = simulation
		self.record_video   = record_video
		
		self.camera_size = camera_size
		
		self._camera_matrix = camera_matrix
		self._camera_distortion = camera_distortion
		
		self.is_detected    = False
		self._kill          = False
		
		#--- 180 deg rotation matrix around the x axis
		self._R_flip      = np.zeros((3,3), dtype=np.float32)
		self._R_flip[0,0] = 1.0
		self._R_flip[1,1] =-1.0
		self._R_flip[2,2] =-1.0

		#--- Define the aruco dictionary
		self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		self._parameters  = aruco.DetectorParameters_create()

		#--- Capture the videocamera (this may also be a video or a picture)
		# self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
		#-- Set the camera size as the one it was calibrated with
		# self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
		# self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

		#-- Font for the text in the image
		self.font = cv2.FONT_HERSHEY_PLAIN

		self._t_read      = time.time()
		self._t_detect    = self._t_read
		self.fps_read    = 0.0
		self.fps_detect  = 0.0    

		file_str = str(datetime.now().year) + '_' + str(datetime.now().month) + '_' + str(datetime.now().day) + '_' + str(datetime.now().hour) + '_' + str(datetime.now().minute) + '.avi'
		self.out = cv2.VideoWriter(file_str,cv2.VideoWriter_fourcc('X','V','I','D'), 20, (self.camera_size[0],self.camera_size[1]))

	def imageCb(self, msg):
		self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

	def runCb(self, data):
		self.run = data.data

	def _rotationMatrixToEulerAngles(self,R):
	# Calculates rotation matrix to euler angles
	# The result is the same as MATLAB except the order
	# of the euler angles ( x and z are swapped ).
	
		def isRotationMatrix(R):
			Rt = np.transpose(R)
			shouldBeIdentity = np.dot(Rt, R)
			I = np.identity(3, dtype=R.dtype)
			n = np.linalg.norm(I - shouldBeIdentity)
			return n < 1e-6        
		assert (isRotationMatrix(R))

		sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

		singular = sy < 1e-6

		if not singular:
			x = math.atan2(R[2, 1], R[2, 2])
			y = math.atan2(-R[2, 0], sy)
			z = math.atan2(R[1, 0], R[0, 0])
		else:
			x = math.atan2(-R[1, 2], R[1, 1])
			y = math.atan2(-R[2, 0], sy)
			z = 0

		return np.array([x, y, z])

	def _update_fps_read(self):
		t           = time.time()
		self.fps_read    = 1.0/(t - self._t_read)
		self._t_read      = t
		
	def _update_fps_detect(self):
		t           = time.time()
		self.fps_detect  = 1.0/(t - self._t_detect)
		self._t_detect      = t    

	def stop(self):
		self._kill = True

	def track(self, loop=True, verbose=True, show_video=None):
		self._kill = False
		if show_video is None: show_video = self._show_video
		
		marker_found = False
		x = y = z = 0
		rate = rospy.Rate(30)


		pub_coord = rospy.Publisher('aruco_coord', Vector3, queue_size=1)
		coord = Vector3()

		pub_vis = rospy.Publisher('aruco_visible', Bool, queue_size=1)

		while not self._kill and not rospy.is_shutdown():

			if self.run:
			
				#-- Read the camera frame
				if(self.simulation):
					frame = self.frame

				else:
					frame = self.frame
					#ret_vid, frame = self._cap.read()

				self._update_fps_read()
				
				#-- Convert in gray scale
				gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

				#-- Find all the aruco markers in the image
				corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
								parameters=self._parameters,
								distCoeff=self._camera_distortion,
								cameraMatrix=self._camera_matrix)
				
				if not ids is None and self.id_to_find in ids[0]:
					marker_found = True
					self._update_fps_detect()
					#-- ret = [rvec, tvec, ?]
					#-- array of rotation and position of each marker in camera frame
					#-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
					#-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
					ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)

					#-- Unpack the output, get only the first
					rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
					
					x = tvec[0]
					y = tvec[1]
					z = tvec[2]

					#-- Draw the detected marker and put a reference frame over it

					#-- Obtain the rotation matrix tag->camera
					R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
					R_tc    = R_ct.T

					#-- Get the attitude in terms of euler 321 (Needs to be flipped first)
					roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)

					#-- Now get Position and attitude f the camera respect to the marker
					pos_camera = -R_tc*np.matrix(tvec).T
					
					# print "Camera X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(pos_camera[0], pos_camera[1], pos_camera[2],fps_detect)
					if verbose: print("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(tvec[0], tvec[1], tvec[2],self.fps_detect))

					if show_video or self.record_video:
						aruco.drawDetectedMarkers(frame, corners)
						aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

						#-- Print the tag position in camera frame
						str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
						cv2.putText(frame, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)        
						
						#-- Print the marker's attitude respect to camera frame
						str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
											math.degrees(yaw_marker))
						cv2.putText(frame, str_attitude, (0, 150), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

						str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
						cv2.putText(frame, str_position, (0, 200), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

						#-- Get the attitude of the camera respect to the frame
						roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
						str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
											math.degrees(yaw_camera))
						cv2.putText(frame, str_attitude, (0, 250), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)


				else:
					x = 0
					y = 0
					z = 0
					marker_found = False
					if verbose: print("Nothing detected - fps = %.0f"%self.fps_read)
				

				if show_video:
					#--- Display the frame
					dim = (160,120)
					frame_new = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
					#cv2.imwrite(file,)
					cv2.imshow('frame', frame_new)

							#--- use 'q' to quit
					key = cv2.waitKey(1) & 0xFF
					if key == ord('q'):
						self._cap.release()
						cv2.destroyAllWindows()

				if self.record_video:
					self.out.write(frame)

		
				coord.x = x
				coord.y = y
				coord.z = z
				pub_coord.publish(coord)
				pub_vis.publish(data=marker_found)

			else:

				coord.x = 0
				coord.y = 0
				coord.z = 0
				pub_coord.publish(coord)
				pub_vis.publish(False)


			rate.sleep()
   
			#if not loop: return(marker_found, x, y, z, corners)
			if not loop: return()
			
if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='camera_param')
	parser.add_argument('--camera_topic', default='')
	parser.add_argument('--drone_ID', default='')
	args = parser.parse_args(rospy.myargv()[1:])
	camera_topic = args.camera_topic
	drone_ID = args.drone_ID

	#--- Define Tag
	id_to_find  = 70
	marker_size  = 50 #- [cm]

	#--- Get the camera calibration path
	calib_path  = os.path.dirname(os.path.abspath(__file__))
	camera_matrix   = np.loadtxt(calib_path+'/cameraMatrix.txt', delimiter=',')
	camera_distortion   = np.loadtxt(calib_path+'/cameraDistortion.txt', delimiter=',')                                      
	aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, camera_matrix=camera_matrix, camera_distortion=camera_distortion, record_video=False, simulation=True, camera_topic=camera_topic, drone_ID=drone_ID)
	aruco_tracker.track(loop = True)

