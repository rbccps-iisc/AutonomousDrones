#!/usr/bin/python

from __future__ import print_function
from os import sys, path
import select, termios, tty, rospy, argparse, mavros, threading, time, readline, signal, tf, quadprog, math, tf2_ros, tf2_geometry_msgs, csv

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from datetime import datetime
from opencv.lib_aruco_pose import ArucoSingleTracker
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from rosgraph_msgs.msg import Clock

from mavros import command
from mavros_msgs.msg import PositionTarget, Altitude, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from math import pow, sqrt

from gazebo_msgs.msg import ModelStates, ContactsState

from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import numpy as np
from MPC import MPC_solver

global R
global roll, pitch, yaw

hz                                      = 5.0
n                                       = 15
t                                       = 1/hz
print(t, hz)
gps_rate                                = 0
cont                                    = 0
home_en_recorded = home_u_recorded      = False
cart_e = cart_n = cart_u                = 0.0
vel_e = vel_n = vel_u                   = 0.0
home_e = home_n = home_u                = 0.0
ekf_e = ekf_n = ekf_u                   = 0
desired_e = desired_n = desired_u       = 0.0
limit_e = limit_n = limit_u             = 220
roll = pitch = yaw                      = 0.0
TIMEOUT                                 = 0.5
kp                                      = 1.
kb                                      = 10000000.0
home_yaw                                = 0
br                                      = tf.TransformBroadcaster()
# br2                                     = tf.TransformBroadcaster()
n_pub                                   = rospy.Publisher('n_graph', Float32, queue_size = 5)
discard_samples                         = 20                        #samples to discard before gps normalizes
pos                                     = Point()
quat                                    = Quaternion()
pos.x = pos.y = pos.z                   = 0
quat.x = quat.y = quat.z = quat.w       = 0
start_n                                 = 0.0
start_time                              = rospy.Time()
cached_var                              = {}
flag                                    = True
max_acc                                 = 0
detected_aruco                          = False
aruco_e = aruco_n = aruco_u             = 0
prev_vel                                = 0
prev_time                               = rospy.Time()
time_taken                              = 0
start_clock                             = False
contact_made                            = False
init_pose                               = False
gz_e = gz_n = gz_u = e_0 = n_0          = 0

id_to_find          = 72
marker_size         = 50
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_distortion   = np.loadtxt(calib_path+'cameraDistortionSim.txt', delimiter=',')                                      
camera_matrix       = np.loadtxt(calib_path+'cameraMatrixSim.txt', delimiter=',')

file_str = '_' + str(datetime.now().month) + '_' + str(datetime.now().day) + '_' + str(datetime.now().hour) + '_' + str(datetime.now().minute) + '.csv'

if not path.isfile(file_str):
    csvfile = open(file_str,'w')
    fieldnames = ['Time','cart_e','cart_n','cart_u','vel_e','vel_n','vel_u','desired_e','desired_n','desired_u']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    csvfile.close()

csvfile = open(file_str,'a')

writer = csv.writer(csvfile)


def clamp(num, value):
   return max(min(num, value), -value)


def get_pos_cb(data):
    global gz_e, gz_n, gz_u, e_0, n_0, init_pose, vel_e, vel_n, vel_u

    gz_e = data.pose[0].position.x
    gz_n = data.pose[0].position.y
    gz_u = data.pose[0].position.z

    # vel_e = -data.twist[0].linear.y
    # vel_n = data.twist[0].linear.x
    # vel_u = data.twist[0].linear.z
 
    if not init_pose:
        e_0 = gz_e
        n_0 = gz_n
        init_pose = True


def contact_cb(data):
    global contact_made, start_time

    contact = data.states
    current_time = data.header.stamp

    # print("NO CONTACT")
    if contact.__len__()!=0 and not contact_made:
        print("CONTACT!!!!!!!!!!!!!!!!")
        contact_made = True
        contact_force = contact[0].total_wrench.force.z
        r_0 = sqrt(pow(e_0,2)+pow(n_0,2))
        e_err = gz_e
        n_err = gz_n
        r_err = sqrt(pow(e_err,2)+pow(n_err,2))
        print(current_time, start_time)
        t = (current_time-start_time).to_sec()
        f = contact[0].total_wrench.force.z
        writer.writerow([int(aruco_e), int(aruco_n), int(e_0*100), int(n_0*100), int(r_0*100), int(e_err*100), int(n_err*100), int(r_err*100), t, int(f)])
        csvfile.close() 


def clock_cb(data):
    global start_clock, land_clock, current_time
    
    current_time = data.clock.secs
    
    # if not start_clock:
    #     start_time = current_time
    #     start_clock = True


def imu_cb(data):
    global roll, pitch, yaw, max_acc, br
    orientation_q = data.orientation
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, yaw) = (roll * 180.0/3.1416, pitch * 180.0/3.1416, yaw  * 180.0/3.1416)

    acc = data.linear_acceleration.x

    br.sendTransform((0, 0, 0), (-orientation_q.x, -orientation_q.y, -orientation_q.z, orientation_q.w), rospy.Time.now(), "base_link_att_comp", "base_link")

    if(abs(acc) > max_acc):
        max_acc = abs(acc)
        # print("Measured max:\t",acc)


def gps_local_cb(data):
    global cart_e, cart_n, cart_u, home_e, home_n, home_en_recorded, discard_samples, desired_e, desired_n, start_n

    cart_e = data.pose.pose.position.x
    cart_n = data.pose.pose.position.y
    # cart_u = data.pose.pose.position.z

    if home_en_recorded is False and cart_e != 0 and cart_n != 0:
        home_e = cart_e
        home_n = cart_n
        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            desired_e = cart_e                          #to set home position as initial desired position
            desired_n = cart_n
            start_n = home_n
            home_en_recorded = True


def pose_cb(data):
    global ekf_e, ekf_n,ekf_u

    position = data.pose.position
    ekf_e = position.x
    ekf_n = position.y
    ekf_u = position.z


def velocity_cb(data):
    global vel_e, vel_n, vel_u, prev_time, prev_vel, max_acc, delta_time

    vel_e = data.twist.linear.x
    vel_n = data.twist.linear.y
    vel_u = data.twist.linear.z

    curr_time = data.header.stamp
    delta_time = (curr_time - prev_time).to_sec()# * 1e-9
    # print(delta_time, curr_time, prev_time)
    # print(type(delta_time), type(curr_time), type(prev_time))
    acc = (vel_e - prev_vel)/(delta_time)

    prev_vel = vel_e
    prev_time = curr_time
    # print(prev_time, type(prev_time))
    if(abs(acc) > max_acc):
        max_acc = abs(acc)
        # print("Measured max:\t",acc)


def calc_target_cb(data):
    global desired_e, desired_n, desired_u, flag, home_e, home_n, home_u, flag

    flag = not flag

    if(flag):
        desired_e = home_e + data.pose.position.x
        desired_n = home_n + data.pose.position.y 
        desired_u = home_u + 10
        print("UP")

    else:
        desired_e = 0
        desired_n = 0
        desired_u = home_u + 1
        print("DOWN")


def twist_obj(x, y, z, a, b, c):
    # movx_cmd = Twist()
    movx_cmd = TwistStamped(header=Header(stamp=rospy.get_rostime()))
    movx_cmd.twist.linear.x = x
    movx_cmd.twist.linear.y = y
    movx_cmd.twist.linear.z = z
    movx_cmd.twist.angular.x = a
    movx_cmd.twist.angular.y = b
    movx_cmd.twist.angular.z = c
    return movx_cmd


def gazebo_cb(data):
    global br
    global cont,rate, pos, quat 

    pos = data.pose[1].position
    quat = data.pose[1].orientation


def armed_cb(data):
    global armed
    armed = data.armed


def range_cb(data):
    global cart_u
    cart_u = data.range


def main():
    global home_en_recorded, home_u_recorded, cart_e, cart_n, cart_u, desired_e, desired_n, desired_u, home_yaw, aruco_e, aruco_n, aruco_u, armed
    global home_e, home_u, home_n, limit_e, limit_n, limit_u, cont, n, t, start_time, cached_var, detected_aruco, time_taken, delta_time, writer, csvfile
    xAnt = yAnt = 0
    acc = 0
    max_acc = 0
    home_en_recorded = False
    rospy.init_node('MAVROS_Listener')
    
    rate = rospy.Rate(hz)

    aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True, camera_distortion=camera_distortion, camera_matrix=camera_matrix, simulation=True)

    tf_buff = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buff)

    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    # rospy.Subscriber("/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber("/mavros/global_position/local", Odometry, gps_local_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
    # rospy.Subscriber("/gazebo/model_states", ModelStates, get_pos_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_cb)
    # rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, velocity_cb)
    rospy.Subscriber("/mavros/state", State, armed_cb)
    rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, range_cb)

    #time subscriber
    rospy.Subscriber('clock', Clock, clock_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
    pub2 = rospy.Publisher('gps_point', PointStamped, queue_size = 5)
    pub3 = rospy.Publisher('boundarn_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('path', Path, queue_size=1)
    pub5 = rospy.Publisher('ekf_path', Path, queue_size=1)
    pub6 = rospy.Publisher('mpc_path', Path, queue_size=1)

    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    set_landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    path = Path()
    ekf_path = Path()
    mpc_path = Path()
    mae_append = 1000

    last_vel = 0

    data_timer = 0.
    hold_timer = 0.

    # time.sleep(20)

    mavros.command.arming(True)
    set_mode(0, 'GUIDED')
    if(cart_u < 5):

        set_takeoff(0, 0, None, None, 5.5)
        # set_mode(0, 'AUTO.TAKEOFF')

    while cart_u < 5 and not rospy.is_shutdown(): continue

    #contact force subscriber
    # rospy.Subscriber('/bumper_states', ContactsState, contact_cb)

    # for i in range(0, 10):
    #     pub1.publish(twist_obj(0, 0, 0, 0.0, 0.0, 0.0))
    #     time.sleep(0.01)

    # set_mode(0, 'OFFBOARD')

    while not rospy.is_shutdown():
        marker_found, x_cm, y_cm, z_cm, _ = aruco_tracker.track(loop=False)

    	# yaw = 360.0 + yaw if yaw < 0 else yaw

        if home_u_recorded is False and cart_u != 0 and yaw != 0:
            # desired_u = cart_u + 3
            home_yaw = yaw
            home_u = cart_u
            home_u_recorded = True


        # +90 To account for diff between world and drone frame
        # desired_yaw = (math.atan2(desired_n - cart_n, desired_e - cart_e) * 180 / 3.1416)
        # desired_yaw = 360.0 + desired_yaw if desired_yaw < 0 else desired_yaw

        aruco_cam_pos = tf2_geometry_msgs.PointStamped(header=Header(stamp=rospy.Time.now(), frame_id='base_link'))
        aruco_cam_pos.point.x = y_cm/100
        aruco_cam_pos.point.y = x_cm/100
        aruco_cam_pos.point.z = z_cm/100

        try:
            p = tf_buff.transform(aruco_cam_pos, "base_link_att_comp", timeout=rospy.Duration(0.2))

        except :
            print('Oops')
            p.point.x = aruco_e
            p.point.y = aruco_n
            p.point.z = aruco_u

        ################################ MPC ###################################
        if(marker_found):
            detected_aruco = True

            aruco_e = p.point.x
            aruco_n = p.point.y
            aruco_u = p.point.z
            velocity_e_des, cached_var, diff = MPC_solver(aruco_e, 0, limit_e, 0, n, t, True, variables = cached_var, vel_limit = 0.1, acc=0.5, curr_vel=vel_e)
            e_array = cached_var.get("points")
            velocity_n_des, cached_var, _ = MPC_solver(aruco_n, 0, limit_n, 0, n, t, True, variables = cached_var, vel_limit = 0.1, acc=0.5, curr_vel=vel_n)
            n_array = cached_var.get("points")
            velocity_u_des, cached_var, _ = MPC_solver(aruco_u, 3, limit_u, 0, n, t, True, variables = cached_var, vel_limit = 0.1, acc=0, curr_vel=vel_u, debug=False)
            u_array = cached_var.get("points")
            mpc_point_arr = np.transpose(np.row_stack((e_array, n_array, u_array)))

            acc = diff / t
            # print("Marker seen\t", aruco_e, -y_cm, velocity_e_des)


        elif(not marker_found and not detected_aruco):
            start_time = rospy.Time.now()

            velocity_e_des = -0
            velocity_n_des = 0
            velocity_u_des = 0
        # print(p)
        
        # else:
        #     velocity_e_des, cached_var, diff = MPC_solver(cart_e, aruco_e, limit_e, 0, n, t, True, variables = cached_var, vel_limit = 0.5, acc = 1.8, curr_vel=vel_e)
        #     e_array = cached_var.get("points")
        #     velocity_n_des, cached_var, _ = MPC_solver(cart_n, aruco_n, limit_n, 0, n, t, True, variables = cached_var, vel_limit = 0.5, acc = 1.8, curr_vel=vel_n)
        #     n_array = cached_var.get("points")
        #     velocity_u_des, cached_var, _ = MPC_solver(cart_u, aruco_u, limit_u, 0, n, t, True, variables = cached_var, vel_limit = 1.5, acc = 0, curr_vel=vel_u)
        #     u_array = cached_var.get("points")
        #     velocity_u_des = 0
        #     # print("Marker unseen\t",aruco_e, velocity_e_des)

        #     mpc_point_arr = np.transpose(np.row_stack((e_array, n_array, u_array)))
        
        print("Generated vel:\t",velocity_u_des,"Current vel:\t", vel_u)
        # print("Orig E:\t", y_cm, "Orig N:\t", x_cm)
        print("Aruco E:\t", aruco_e, "Aruco N:\t", aruco_n, "Aruco U:\t", aruco_u)
        # velocity_e_des = clamp(velocity_e_des, 1.5)
        # velocity_n_des = clamp(velocity_n_des, 1.5)
        # velocity_u_des = clamp(velocity_u_des, 1.5)

        data_timer = data_timer + delta_time

        if(data_timer > 0.1):
            writer.writerow([rospy.get_time(), float(cart_e), float(cart_n), float(cart_u), float(vel_e), float(vel_n), float(vel_u), float(velocity_e_des), float(velocity_n_des), float(velocity_u_des)])
            data_timer = 0.

        dist = sqrt((aruco_e)**2+(aruco_n)**2)#+(aruco_u)**2)

        if(dist < 0.25 and marker_found == True and cart_u < 3.25):
            hold_timer = hold_timer + delta_time

            velocity_e_des = velocity_n_des = 0

            print("Time to land:\t", hold_timer)
            # if(hold_timer > 3):
            set_mode(0, 'LAND')
            csvfile.close()

            sys.exit()

        pub1.publish(twist_obj(velocity_e_des, velocity_n_des, velocity_u_des, 0.0, 0.0, 0.0))
        # pub1.publish(twist_obj(velocity_e_des, 0, 0, 0.0, 0.0, 0.0))

        if(acc > abs(max_acc)):
            max_acc = abs(acc)
            # print("MPC Max:",max_acc)

        # if(cart_u < 1):
        #     # set_landing(0, 0, None, None, 0)

        #     # set_mode(0, 'AUTO.LAND')
        #     set_mode(0, 'LAND')

        #     # print(time.time() - start_time)
        #     if(not armed):
        #         sys.exit()

        if(detected_aruco):
            desired_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
            desired_point.header.frame_id = 'map'
            desired_point.point.x = -y_cm/100
            desired_point.point.y = -x_cm/100
            desired_point.point.z = z_cm/100
            pub.publish(desired_point)

            gps_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
            gps_point.header.frame_id = 'map'
            gps_point.point.x = aruco_e
            gps_point.point.y = aruco_n
            gps_point.point.z = aruco_u
            pub2.publish(gps_point)

            ekf_pose = PoseStamped()
            ekf_pose.header.frame_id = "map"
            ekf_pose.pose.position.x = ekf_e
            ekf_pose.pose.position.y = ekf_n
            ekf_pose.pose.position.z = ekf_u

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = pos.x
            pose.pose.position.y = pos.y
            pose.pose.position.z = pos.z

            
            mpc_pose_array = [None] * n
            for i in range(0, n):
                mpc_pose = PoseStamped()
                mpc_pose.header.seq = i
                mpc_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * 1)
                mpc_pose.header.frame_id = "map"
                mpc_pose.pose.position.x = mpc_point_arr[i][0]# + desired_e - home_e
                mpc_pose.pose.position.y = mpc_point_arr[i][1]# + desired_n - home_n
                mpc_pose.pose.position.z = mpc_point_arr[i][2]# + desired_u - home_u
                mpc_pose_array[i] = mpc_pose

            # if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
            pose.header.seq = path.header.seq + 1
            path.header.frame_id = "map"
            path.header.stamp = rospy.Time.now()
            pose.header.stamp = path.header.stamp
            path.poses.append(pose)
            ekf_pose.header.seq = ekf_path.header.seq + 1
            ekf_path.header.frame_id = "map"
            ekf_path.header.stamp = rospy.Time.now()
            ekf_pose.header.stamp = ekf_path.header.stamp
            ekf_path.poses.append(ekf_pose)
            # mpc_pose.header.seq = ekf_path.header.seq + 1
            mpc_path.header.frame_id = "map"
            mpc_path.header.stamp = rospy.Time.now()
            # mpc_pose.header.stamp = mpc_path.header.stamp
            mpc_path.poses = mpc_pose_array
            cont = cont + 1

            xAnt = pose.pose.orientation.x
            yAnt = pose.pose.position.y

            pub4.publish(path)
            pub5.publish(ekf_path)
            pub6.publish(mpc_path)

            if cont > mae_append and len(path.poses) != 0 and len(ekf_path.poses):
                    path.poses.pop(0)
                    ekf_path.poses.pop(0)

        rate.sleep()

    csvfile.close()

        # br2.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "fcu", "map")

        
if __name__ == "__main__":
    mavros.set_namespace("/mavros")
    path = Path() 
    np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    main()
