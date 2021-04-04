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

global R
global roll, pitch, yaw

hz                                      = 10.0
n                                       = 15
t                                       = 1/hz
print(t, hz)
gps_rate                                = 0
cont                                    = 0
home_xy_recorded = home_z_recorded      = False
cart_x = cart_y = cart_z                = 0.0
vel_x = vel_y = vel_z                   = 0.0
home_x = home_y = home_z                = 0.0
ekf_x = ekf_y = ekf_z                   = 0
desired_x = desired_y = desired_z       = 0.0
limit_x = limit_y = limit_z             = 220
roll = pitch = yaw                      = 0.0
TIMEOUT                                 = 0.5
kp                                      = 1.
kb                                      = 10000000.0
home_yaw                                = 0
br                                      = tf.TransformBroadcaster()
# br2                                     = tf.TransformBroadcaster()
y_pub                                   = rospy.Publisher('y_graph', Float32, queue_size = 5)
discard_samples                         = 20                        #samples to discard before gps normalizes
pos                                     = Point()
quat                                    = Quaternion()
pos.x = pos.y = pos.z                   = 0
alt                                     = 0.0
quat.x = quat.y = quat.z = quat.w       = 0
start_y                                 = 0.0
start_time                              = rospy.Time()
cached_var                              = {}
cached_var_y                            = {}
cached_var_z                            = {}
flag                                    = True
max_acc                                 = 0
detected_aruco                          = False
aruco_x = aruco_y = aruco_z             = 0
prev_vel                                = 0
prev_time                               = rospy.Time()
time_taken                              = 0
start_clock                             = False
is_reached                              = False
init_pose                               = False
gz_x = gz_y = gz_z = x_0 = y_0          = 0
full_avg                                = 0.
vision_avg                              = 0.
coord                                   = [0,0,0]
visible                                 = False

id_to_find          = 72
marker_size         = 20
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')                                      
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')


file_str = path.dirname(path.abspath(__file__)) +'/_' + str(datetime.now().month) + '_' + str(datetime.now().day) + '_' + str(datetime.now().hour) + '_' + str(datetime.now().minute) + '.csv'


if not path.isfile(file_str):
    csvfile = open(file_str,'w')
    fieldnames = ['Time','cart_x','cart_y','cart_z','vel_x','vel_y','vel_z','desired_x','desired_y','desired_z','aruco_x','aruco_y']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    csvfile.close()

csvfile = open(file_str,'a')
writer = csv.writer(csvfile)


def clamp(num, value):
   return max(min(num, value), -value)


def get_pos_cb(data):
    global gz_x, gz_y, gz_z, x_0, y_0, init_pose

    gz_x = data.pose[2].position.x
    gz_y = data.pose[2].position.y
    gz_z = data.pose[2].position.z
 
    if not init_pose:
        x_0 = gz_x
        y_0 = gz_y
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
        r_0 = sqrt(pow(x_0,2)+pow(y_0,2))
        x_xrr = gz_x
        y_xrr = gz_y
        r_xrr = sqrt(pow(x_xrr,2)+pow(y_xrr,2))
        # print(current_time, start_time)
        t = (current_time-start_time).to_sec()
        f = contact[0].total_wrench.force.z
        #writer.writerow([int(aruco_x), int(aruco_y), int(x_0*100), int(y_0*100), int(r_0*100), int(x_xrr*100), int(y_xrr*100), int(r_xrr*100), t, int(f)])
        #csvfile.close() 


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

    # if(abs(acc) > max_acc):
    #     max_acc = abs(acc)
    #     print("Measured max:\t",acc)


def gps_local_cb(data):
    global cart_x, cart_y, cart_z, home_x, home_y, home_xy_recorded, discard_samples, desired_x, desired_y, start_y

    cart_x = data.pose.pose.position.x
    cart_y = data.pose.pose.position.y
    cart_z = data.pose.pose.position.z

    if home_xy_recorded is False and cart_x != 0 and cart_y != 0:
        home_x = cart_x
        home_y = cart_y
        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            desired_x = cart_x                          #to set home position as initial desired position
            desired_y = cart_y
            start_y = home_y
            home_xy_recorded = True


def pose_cb(data):
    global ekf_x, ekf_y,ekf_z

    position = data.pose.position
    ekf_x = position.x
    ekf_y = position.y
    ekf_z = position.z

def alt_cb(data):
    global alt, sub_alt
    alt = data.amsl
    sub_alt.unregister()

def velocity_cb(data):
    global vel_x, vel_y, vel_z, prev_time, prev_vel, max_acc, delta_time

    vel_x = data.twist.linear.x
    vel_y = data.twist.linear.y
    vel_z = data.twist.linear.z

    curr_time = data.header.stamp
    delta_time = (curr_time - prev_time).to_sec()# * 1e-9
    #print(delta_time, curr_time, prev_time)
    # print(type(delta_time), type(curr_time), type(prev_time))
    #acc = (vel_x - prev_vel)/(delta_time)

    prev_vel = vel_x
    prev_time = curr_time
    # print(prev_time, type(prev_time))
'''    if(abs(acc) > max_acc):
        max_acc = abs(acc)
        print("Measured max:\t",acc)
'''

def calc_target_cb(data):
    global desired_x, desired_y, desired_z, flag, home_x, home_y, home_z, flag

    flag = not flag

    if(flag):
        desired_x = home_x + data.pose.position.x
        desired_y = home_y + data.pose.position.y 
        desired_z = home_z + 10
        print("UP")

    else:
        desired_x = 0
        desired_y = 0
        desired_z = home_z + 1
        print("DOWN")


def twist_obj(x, y, z, a, b, c):
    # move_cmd = Twist()
    move_cmd = TwistStamped(header=Header(stamp=rospy.get_rostime()))
    move_cmd.twist.linear.x = x
    move_cmd.twist.linear.y = y
    move_cmd.twist.linear.z = z
    move_cmd.twist.angular.x = a
    move_cmd.twist.angular.y = b
    move_cmd.twist.angular.z = c
    return move_cmd


def gazebo_cb(data):
    global br
    global cont,rate, pos, quat 

    pos = data.pose[1].position
    quat = data.pose[1].orientation


def armed_cb(data):
    global armed
    armed = data.armed

def aruco_coord_cb(data):
    global coord
    coord[0] = data.x
    coord[1] = data.y
    coord[2] = data.z

def aruco_visible_cb(data):
    global visible
    visible = data.data

def range_cb(data):
    global cart_z
    cart_z = data.range


def geo_pose_obj(x, y, z, a, b, c, d):
    # move_cmd = Twist()
    pose_cmd = GeoPoseStamped(header=Header(stamp=rospy.get_rostime()))
    pose_cmd.pose.position.latitude = x
    pose_cmd.pose.position.longitude = y
    pose_cmd.pose.position.altitude = z
    pose_cmd.pose.orientation.x = a
    pose_cmd.pose.orientation.y = b
    pose_cmd.pose.orientation.z = c
    pose_cmd.pose.orientation.w = d
    return pose_cmd


def main(drone_ID='nan', home_lat=13.0272156, home_lon=77.5638397, call=False, first_land=True):
    global home_xy_recorded, home_z_recorded, cart_x, cart_y, cart_z, desired_x, desired_y, desired_z, home_yaw, aruco_x, aruco_y, aruco_z, armed, alt
    global home_x, home_z, home_y, limit_x, limit_y, limit_z, cont, n, t, start_time, cached_var, cached_var_y, cached_var_z, time_taken
    global vision_avg, full_avg, coord, visible, detected_aruco, is_reached, sub_alt

    xAnt = yAnt = 0
    acc = 0
    max_acc = 0
    home_xy_recorded = False
    
    if not call:
        rospy.init_node('MAVROS_Listener')
    
    rate = rospy.Rate(hz)

    # aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, camera_distortion=camera_distortion, camera_matrix=camera_matrix, simulation=True, record_video=False)

    tf_buff = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buff)

    if not call:
        rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
        sub_alt = rospy.Subscriber("/mavros/altitude", Altitude, alt_cb)
        rospy.Subscriber("/mavros/global_position/local", Odometry, gps_local_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_cb)
        rospy.Subscriber("/mavros/state", State, armed_cb)

    else:
        if first_land:
            rospy.Subscriber(drone_ID+"/mavros/imu/data", Imu, imu_cb)
            sub_alt = rospy.Subscriber(drone_ID+"/mavros/altitude", Altitude, alt_cb)
            rospy.Subscriber(drone_ID+"/mavros/global_position/local", Odometry, gps_local_cb)
            rospy.Subscriber(drone_ID+"/mavros/local_position/pose", PoseStamped, pose_cb)
            rospy.Subscriber(drone_ID+"/mavros/local_position/velocity_local", TwistStamped, velocity_cb)
            rospy.Subscriber(drone_ID+"/mavros/state", State, armed_cb)

    if first_land:
    
        #rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
        #rospy.Subscriber("/gazebo/model_states", ModelStates, get_pos_cb)
        rospy.Subscriber("/aruco_coord", Vector3, aruco_coord_cb)
        rospy.Subscriber("/aruco_visible", Bool, aruco_visible_cb)
        # rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, range_cb)

        #time subscriber
        rospy.Subscriber('clock', Clock, clock_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub2 = rospy.Publisher('gps_point', PointStamped, queue_size = 5)
    pub3 = rospy.Publisher('boundary_cube', Marker, queue_size = 1)
    pub4 = rospy.Publisher('path', Path, queue_size=1)
    pub5 = rospy.Publisher('ekf_path', Path, queue_size = 1)
    pub6 = rospy.Publisher('mpc_path', Path, queue_size = 1)

    if not call:
        pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
        pub7 = rospy.Publisher('/drone1/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)
    else:
        pub1 = rospy.Publisher(drone_ID+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
        pub7 = rospy.Publisher(drone_ID+'/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

    if not call:
        set_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        set_takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        set_landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    else:
        set_mode = rospy.ServiceProxy(drone_ID+'/mavros/set_mode', SetMode)
        set_landing = rospy.ServiceProxy(drone_ID+'/mavros/cmd/land', CommandTOL)


    path = Path()
    ekf_path = Path()
    mpc_path = Path()
    max_append = 1000

    last_vel = 0

    data_timer = 0.
    hold_timer = 0.
    hover_timer = 0.


    if not call:

        mavros.command.arming(True)
    
    #set_mode(0, 'OFFBOARD')

        #print(cart_z)
        if(cart_z < 1):
            set_mode(0, 'AUTO.TAKEOFF')

        while cart_z < 9.5: continue

    #contact force subscriber
    # rospy.Subscriber('/bumper_states', ContactsState, contact_cb)
    
    while not rospy.is_shutdown():
        cont = cont + 1

        start_timer = time.time()

        #marker_found, x_cm, y_cm, z_cm, _ = aruco_tracker.track(loop=False)

        x_cm,y_cm,z_cm = coord
        marker_found = visible

        # print("Vision Time =\t", time.time()-start_timer)
        # vision_avg = (vision_avg + time.time()-start_timer)
        # yaw = 360.0 + yaw if yaw < 0 else yaw

        if home_z_recorded is False and cart_z != 0 and yaw != 0:
            # desired_z = cart_z + 3
            home_yaw = yaw
            home_z = cart_z
            home_z_recorded = True

        aruco_cam_pos = tf2_geometry_msgs.PointStamped(header=Header(stamp=rospy.Time.now(), frame_id='base_link'))
        aruco_cam_pos.point.x = y_cm/100
        aruco_cam_pos.point.y = x_cm/100
        aruco_cam_pos.point.z = z_cm/100

        try:
            p = tf_buff.transform(aruco_cam_pos, "base_link_att_comp", timeout=rospy.Duration(0.005))

        except :
            print('Oops')

        ################################ MPC ###################################
        if(marker_found and is_reached):
            detected_aruco = True
            #print("----------------------------SEEN------------------------------------")
            aruco_x = p.point.x
            aruco_y = p.point.y
            aruco_z = p.point.z
            velocity_x_des, cached_var, diff = MPC_solver(aruco_x, 0, limit_x, 0, n, t, True, variables = cached_var, vel_limit = 0.5, acc=2, curr_vel=vel_x)
            # x_array = cached_var.get("points")
            velocity_y_des, cached_var_y, _ = MPC_solver(aruco_y, 0, limit_y, 0, n, t, True, variables = cached_var_y, vel_limit = 0.5, acc=2, curr_vel=vel_y)
            # y_array = cached_var_y.get("points")
            velocity_z_des, cached_var_z, _ = MPC_solver(aruco_z, 0, limit_z, 0, n, t, True, variables = cached_var_z, vel_limit = 0.5, acc=0, curr_vel=vel_z, pos_cost=1, vel_cost=1000, debug=False)
            # z_array = cached_var_z.get("points")
            # velocity_z_des = -(min(math.sqrt(2*0.064*(aruco_z)), 0.8))
            #print(velocity_z_des)
            pub1.publish(twist_obj(velocity_x_des, velocity_y_des, velocity_z_des, 0.0, 0.0, 0.0))
            # mpc_point_arr = np.transpose(np.row_stack((x_array, y_array, z_array)))

            acc = diff / t


        elif(not is_reached):
            start_time = rospy.Time.now()
            #print("----------------------------NOT SEEN------------------------------------")
            
            if not call:
                alt = alt+10
            
            pub7.publish(geo_pose_obj(home_lat, home_lon, alt, 0, 0, 0, 1))

            if(abs(vel_x) < 0.1 and abs(vel_y) < 0.1):
                hover_timer = hover_timer + delta_time

                #print(hover_timer)

                if(hover_timer > 1):
                    is_reached = True

            # velocity_x_des = -0.5
            # velocity_y_des = 0
            # velocity_z_des = 0
        
        else:
            if(aruco_x < 0):
                velocity_x_des = 0.1

            else:
                velocity_x_des = -0.1

            if(aruco_y < 0):
                velocity_y_des = 0.1

            else:
                velocity_y_des = -0.1

            velocity_z_des = 0

            pub1.publish(twist_obj(velocity_x_des, velocity_y_des, velocity_z_des, 0.0, 0.0, 0.0))


        #     velocity_x_des, cached_var, diff = MPC_solver(cart_x, aruco_x, limit_x, 0, n, t, True, variables = cached_var, vel_limit = 100, acc=1, curr_vel=vel_x)
        #     x_array = cached_var.get("points")
        #     velocity_y_des, cached_var, _ = MPC_solver(cart_y, aruco_y, limit_y, 0, n, t, True, variables = cached_var, vel_limit = 100, acc=1, curr_vel=vel_y)
        #     y_array = cached_var.get("points")
        #     velocity_z_des, cached_var, _ = MPC_solver(cart_z, aruco_z, limit_z, 0, n, t, True, variables = cached_var, vel_limit = 100, acc=1.5, curr_vel=vel_z)
        #     z_array = cached_var.get("points")
        #     # print("Marker unseen\t",aruco_x, velocity_x_des)

        #     mpc_point_arr = np.transpose(np.row_stack((x_array, y_array, z_array)))

        data_timer = data_timer + delta_time
        '''
        if(data_timer > 0.1 and detected_aruco):
            writer.writerow([rospy.get_time(), float(cart_x), float(cart_y), float(cart_z), float(vel_x), float(vel_y), float(vel_z), float(velocity_x_des), float(velocity_y_des), float(velocity_z_des), float(aruco_x), float(aruco_y)])
            data_timer = 0.
        '''
        # dist = sqrt((aruco_x)**2+(aruco_y)**2)#+(aruco_u)**2)

        if(marker_found == True and cart_z <= 1.5):
            velocity_x_des = velocity_y_des = 0

            print("Time to land:\t", hold_timer)

            is_reached = False
            detected_aruco = False
            
            # if(hold_timer > 3):
            set_mode(0, 'AUTO.LAND')
            csvfile.close()

            if not call:
                sys.exit()
            else:
                while armed: 
                    continue
                return
        
        # print(velocity_z_des, vel_z, aruco_z)
        # print("Aruco X:\t", aruco_x, "Aruco Y:\t", aruco_y, "Aruco Z:\t", aruco_z)
        # print("")

        # if(acc > abs(max_acc)):
        #     max_acc = abs(acc)
        #     print("MPC Max:",max_acc)

        # if(detected_aruco):
        #     desired_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        #     desired_point.header.frame_id = 'map'
        #     desired_point.point.x = -y_cm/100
        #     desired_point.point.y = -x_cm/100
        #     desired_point.point.z = z_cm/100
        #     pub.publish(desired_point)

        #     gps_point = PointStamped(header=Header(stamp=rospy.get_rostime()))
        #     gps_point.header.frame_id = 'map'
        #     gps_point.point.x = aruco_x
        #     gps_point.point.y = aruco_y
        #     gps_point.point.z = aruco_z
        #     pub2.publish(gps_point)

        #     ekf_pose = PoseStamped()
        #     ekf_pose.header.frame_id = "map"
        #     ekf_pose.pose.position.x = ekf_x
        #     ekf_pose.pose.position.y = ekf_y
        #     ekf_pose.pose.position.z = ekf_z

        #     pose = PoseStamped()
        #     pose.header.frame_id = "map"
        #     pose.pose.position.x = pos.x
        #     pose.pose.position.y = pos.y
        #     pose.pose.position.z = pos.z

            
        #     mpc_pose_array = [None] * n
        #     for i in range(0, n):
        #         mpc_pose = PoseStamped()
        #         mpc_pose.header.seq = i
        #         mpc_pose.header.stamp = rospy.Time.now() + rospy.Duration(t * 1)
        #         mpc_pose.header.frame_id = "map"
        #         mpc_pose.pose.position.x = mpc_point_arr[i][0]# + desired_x - home_x
        #         mpc_pose.pose.position.y = mpc_point_arr[i][1]# + desired_y - home_y
        #         mpc_pose.pose.position.z = mpc_point_arr[i][2]# + desired_z - home_z
        #         mpc_pose_array[i] = mpc_pose

        #     # if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
        #     pose.header.seq = path.header.seq + 1
        #     path.header.frame_id = "map"
        #     path.header.stamp = rospy.Time.now()
        #     pose.header.stamp = path.header.stamp
        #     path.poses.append(pose)
        #     ekf_pose.header.seq = ekf_path.header.seq + 1
        #     ekf_path.header.frame_id = "map"
        #     ekf_path.header.stamp = rospy.Time.now()
        #     ekf_pose.header.stamp = ekf_path.header.stamp
        #     ekf_path.poses.append(ekf_pose)
        #     # mpc_pose.header.seq = ekf_path.header.seq + 1
        #     mpc_path.header.frame_id = "map"
        #     mpc_path.header.stamp = rospy.Time.now()
        #     # mpc_pose.header.stamp = mpc_path.header.stamp
        #     mpc_path.poses = mpc_pose_array

        #     xAnt = pose.pose.orientation.x
        #     yAnt = pose.pose.position.y

        #     pub4.publish(path)
        #     pub5.publish(ekf_path)
        #     pub6.publish(mpc_path)

            # if cont > max_append and len(path.poses) != 0 and len(ekf_path.poses):
            #         path.poses.pop(0)
            #         ekf_path.poses.pop(0)

        hold_timer = hold_timer + delta_time
        
        if(hold_timer < 0.2):
            set_mode(0, 'OFFBOARD')

        # print("Full Time =\t", time.time()-start_timer)
        full_avg = (full_avg + time.time()-start_timer)

        rate.sleep()

    # print("Vision Average Time =\t", vision_avg/cont)
    print("Full Average Time =\t", full_avg/cont)
        # br2.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "fcu", "map")

        
if __name__ == "__main__":
    mavros.set_namespace("/mavros")
    #pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
    #path = Path() 
    np.set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=1000, suppress=None, nanstr=None, infstr=None, formatter=None)
    main()
