#!/usr/bin/python

from __future__ import print_function
from os import sys, path
import rospy, argparse, mavros, threading, time, signal, tf, quadprog, math, tf2_ros, tf2_geometry_msgs, csv, npyscreen

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, Twist, PointStamped
from rosgraph_msgs.msg import Clock

from datetime import datetime
from mavros import command
from mavros_msgs.msg import PositionTarget, Altitude, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from math import pow, sqrt

from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import numpy as np
from MPC import MPC_solver

global R
global roll, pitch, yaw

desired_e                               = 0
desired_n                               = 0
desired_u                               = 6
hz                                      = 20.0
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
prev_vel                                = 0
prev_time                               = rospy.Time()
time_taken                              = 0
start_clock                             = False
contact_made                            = False
init_pose                               = False
gu_e = gu_n = gu_u = e_0 = n_0          = 0
hold_timer                              = 0.
file_str                                = ''
is_writing                              = False
start_timer				= 0.

id_to_find          = 72
marker_size         = 50
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')                                      
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')

# if not path.isfile('test_land_algo_params_2n_n.csv'):
#     csvfile = open('test_land_algo_params_2n_n.csv','w')
#     fieldnames = ['aruco_e','aruco_n','init_e','init_n','init_r','e_err','n_err','r_err','time_elapsed','contact_force']
#     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#     writer.writeheader()
#     csvfile.close()


# csvfile = open('test_land_algo_params_2n_n.csv','a')

# writer = csv.writer(csvfile)
class form_object(npyscreen.Form):
    def create(self):
        global speed_slider, nsteps_slider, rate_slider, debug, debug2, e_textbox, n_textbox, u_textbox, remarks_textbox, acc_slider
        main_thread.start()

        nsteps_slider = self.add(npyscreen.TitleSlider, name = "No. of steps:", value = 15, out_of = 50)
        rate_slider = self.add(npyscreen.TitleSlider, name = "Rate:", value = 10, out_of = 15, step = 1)
        speed_slider = self.add(npyscreen.TitleSlider, name = "Speed:", value = 0.8, out_of = 3, step = 0.1)
        acc_slider = self.add(npyscreen.TitleSlider, name = "Acceleration XY:", value = 1.8, out_of = 2, step = 0.1)
        self.add(npyscreen.TitleFixedText, name = "Enter Position:")
        # self.add(npyscreen.Textfield, )
        e_textbox = self.add(npyscreen.TitleText, name = "Desired E:", value = "5")
        n_textbox = self.add(npyscreen.TitleText, name = "Desired N:", value = "0")
        u_textbox = self.add(npyscreen.TitleText, name = "Desired Altitude:", value = "6")

        remarks_textbox = self.add(npyscreen.TitleText, name = "Add remarks for data:")

        debug = self.add(npyscreen.TitleText, name="Average time:")
        debug2 = self.add(npyscreen.TitleText, name="Instantaneous time:")

        run_button = self.add(button_object, name = "Run MPC")

    def afterEditing(self):
        global main_thread
        self.parentApp.setNextForm(None)
        main_thread.do_run = False
        main_thread.join()

    def adjust_widgets(self):
        global e_textbox, n_textbox, u_textbox, desired_e, desired_n, desired_u, debug, debug2

        if(e_textbox.value != None and n_textbox.value != None and u_textbox.value != None): 
            desired_e = float(e_textbox.value)
            desired_n = float(n_textbox.value)
            desired_u = float(u_textbox.value)

        debug.value = str(hold_timer)
        debug2.value = str(desired_n)

        debug.display()
        debug2.display()

class App(npyscreen.NPSAppManaged):
    def onStart(self):
        self.addForm('MAIN', form_object, name = "RBCCPS MPC CONTROLLER")

class button_object(npyscreen.Button):
    def whenToggled(self):
        global csvfile, is_writing, writer
        print("Running!")

        if(is_writing == False):

            file_str = remarks_textbox.value + '_' + str(datetime.now().month) + '_' + str(datetime.now().day) + '_' + str(datetime.now().hour) + '_' + str(datetime.now().minute) + '.csv'
            csvfile = open(file_str,'w')
            fieldnames = ['Time','cart_e','cart_n','cart_u','vel_e','vel_n','vel_u','desired_e','desired_n','desired_u']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)#, extrasaction='ignore')
            writer.writeheader()
            csvfile.close()

            is_writing = True

        csvfile = open(file_str,'a')
        writer = csv.writer(csvfile)

        run_thread = threading.Thread(target = run_control, args=())
        run_thread.start()

        # if(run_thread.isAlive()):
        #     value = True

        # else:
        self.value = False

        # run_thread.run()
        # run_control()


def clamp(num, value):
   return max(min(num, value), -value)


def get_pos_cb(data):
    global gu_e, gu_n, gu_u, e_0, n_0, init_pose

    gu_e = data.pose[2].position.x
    gu_n = data.pose[2].position.y
    gu_u = data.pose[2].position.z
 
    if not init_pose:
        e_0 = gu_e
        n_0 = gu_n
        init_pose = True


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
        print("Measured max:\t",acc)


def gps_local_cb(data):
    global cart_e, cart_n, cart_u, home_e, home_n, home_en_recorded, discard_samples, desired_e, desired_n, start_n

    cart_e = data.pose.pose.position.x
    cart_n = data.pose.pose.position.y
    cart_u = data.pose.pose.position.z

    if home_en_recorded is False and cart_e != 0 and cart_n != 0:
        home_e = cart_e
        home_n = cart_n
        discard_samples = discard_samples - 1

        if(discard_samples <= 0):
            # desired_e = cart_e                          #to set home position as initial desired position
            # desired_n = cart_n
            start_n = home_n
            home_en_recorded = True


def contact_cb(data):
    global contact_made, start_time

    contact = data.states
    current_time = data.header.stamp

    # print("NO CONTACT")
    if contact.__len__()!=0 and not contact_made:
        contact_made = True
        contact_force = contact[0].total_wrench.force.z
        r_0 = sqrt(pow(x_0,2)+pow(y_0,2))
        x_err = gz_x
        y_err = gz_y
        r_err = sqrt(pow(x_err,2)+pow(y_err,2))
        # print(current_time, start_time)
        t = (current_time-start_time).to_sec()
        f = contact[0].total_wrench.force.z
        writer.writerow([int(aruco_x), int(aruco_y), int(x_0*100), int(y_0*100), int(r_0*100), int(x_err*100), int(y_err*100), int(r_err*100), t, int(f)])
        csvfile.close() 


def pose_cb(data):
    global ekf_e, ekf_n,ekf_u

    position = data.pose.position
    ekf_e = position.x
    ekf_n = position.y
    ekf_u = position.z


def velocity_cb(data):
    global vel_e, vel_n, vel_u, prev_time, delta_time, prev_vel, max_acc

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
    # move_cmd = Twist()
    move_cmd = TwistStamped(header=Header(stamp=rospy.get_rostime()))
    move_cmd.twist.linear.x = x
    move_cmd.twist.linear.y = y
    move_cmd.twist.linear.z = z
    move_cmd.twist.angular.x = a
    move_cmd.twist.angular.y = b
    move_cmd.twist.angular.z = c
    return move_cmd


def pose_obj(x, y, z, a, b, c, d):
    # move_cmd = Twist()
    pose_cmd = PoseStamped(header=Header(stamp=rospy.get_rostime()))
    pose_cmd.pose.position.x = x
    pose_cmd.pose.position.y = y
    pose_cmd.pose.position.z = z
    pose_cmd.pose.orientation.x = a
    pose_cmd.pose.orientation.y = b
    pose_cmd.pose.orientation.z = c
    pose_cmd.pose.orientation.w = d
    return pose_cmd


def armed_cb(data):
    global armed
    armed = data.armed


def main():
    global pub, pub1, pub2, pub3, pub4, pub5, pub6, pub7, rate, set_arming, set_mode    
    rate = rospy.Rate(hz)

    tf_buff = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buff)

    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    # rospy.Subscriber("/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber("/mavros/global_position/local", Odometry, gps_local_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, calc_target_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_cb)
    rospy.Subscriber("/mavros/state", State, armed_cb)

    pub = rospy.Publisher('destination_point', PointStamped, queue_size = 1)
    pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
    pub2 = rospy.Publisher('gps_point', PointStamped, queue_size = 5)
    pub4 = rospy.Publisher('path', Path, queue_size=1)
    pub5 = rospy.Publisher('ekf_path', Path, queue_size=1)
    pub6 = rospy.Publisher('mpc_path', Path, queue_size=1)
    pub7 = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    set_landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    mavros.command.arming(True)

    time.sleep(0.2)

    set_mode(0, 'GUIDED')

    if(cart_u < 1):
        time.sleep(0.5)
        set_takeoff(0, 0, None, None, 6)
        #set_mode(0, 'AUTO.TAKEOFF')

    while cart_u < 2 and not rospy.is_shutdown() and getattr(main_thread, "do_run", True): continue

    # main_thread = threading.currentThread()
    # main_thread.join()
    # run_control()


def run_control():
    global home_u_recorded, desired_e, desired_n, desired_u, home_yaw, armed, hold_timer, nsteps_slider, speed_slider, acc_slider
    global cont, n, t, start_time, detected_aruco, time_taken, cached_var, main_thread, delta_time, rate_slider, writer, is_writing, start_timer
    xAnt = yAnt = 0
    acc = 0
    max_acc = acc_slider.value
    hold_timer = 0.



    path = Path()
    ekf_path = Path()
    mpc_path = Path()
    max_append = 1000

    vel = float(speed_slider.value)
    n = int(nsteps_slider.value)
    t = 1/float(rate_slider.value)

    data_timer = 0.

    while not rospy.is_shutdown() and getattr(main_thread, "do_run", True):
	start_timer = time.time()
        if home_u_recorded is False and cart_u != 0 and yaw != 0:
            # desired_u = cart_u + 3
            home_yaw = yaw
            home_u = cart_u
            home_u_recorded = True

        ################################ MPC ###################################

        velocity_e_des, cached_var, diff = MPC_solver(cart_e, desired_e, limit_e, 0, n, t, True, variables = cached_var, vel_limit = vel, acc = max_acc, curr_vel=vel_e)
        e_array = cached_var.get("points")
        velocity_n_des, cached_var, _ = MPC_solver(cart_n, desired_n, limit_n, 0, n, t, True, variables = cached_var, vel_limit = vel, acc = max_acc, curr_vel=vel_n)
        n_array = cached_var.get("points")
        velocity_u_des, cached_var, _ = MPC_solver(cart_u, desired_u, limit_u, 0, n, t, True, variables = cached_var, vel_limit = 0.3, acc = 0, curr_vel=vel_u)
        u_array = cached_var.get("points")

        mpc_point_arr = np.transpose(np.row_stack((e_array, n_array, u_array)))
        

        # print("Generated vel:\t",velocity_e_des,"Current vel:\t", vel_e, "Aruco E:\t", aruco_e)
        # print("Current E:\t", cart_e, "Current N:\t", cart_n, "Current U:\t", cart_u)
        # print("Generated Z velocity:\t", velocity_u_des, "Current Z velocity:\t", vel_u)
        # velocity_e_des = clamp(velocity_e_des, 0.5)
        # velocity_n_des = clamp(velocity_n_des, 0.5)
        # velocity_u_des = clamp(velocity_u_des, 0.5)

        pub1.publish(twist_obj(velocity_e_des, velocity_n_des, velocity_u_des, 0.0, 0.0, 0.0))
        # pub1.publish(twist_obj(velocity_e_des, 0, 0, 0.0, 0.0, 0.0))

        dist = sqrt((cart_e - desired_e)**2+(cart_n - desired_n)**2+(cart_u - desired_u)**2)

        if(dist < 1):
            hold_timer = hold_timer + delta_time

            if(hold_timer > 5):
                # set_mode(0, 'GUIDED')
                csvfile.close()

                is_writing = False
                pub7.publish(pose_obj(0, 0, 6, 0, 0, 0, 0))
                sys.exit()
            # print(time.time() - start_time)
            # if(not armed):

        data_timer = data_timer + delta_time

        if(data_timer > 0.1):
            writer.writerow([rospy.get_time(), float(cart_e), float(cart_n), float(cart_u), float(vel_e), float(vel_n), float(vel_u), float(velocity_e_des), float(velocity_n_des), float(velocity_u_des)])
            data_timer = 0.
            # csvfile.close()

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

            if cont > max_append and len(path.poses) != 0 and len(ekf_path.poses):
                    path.poses.pop(0)
                    ekf_path.poses.pop(0)
	
	print(time.time() - start_timer)
        rate.sleep()

        
if __name__ == "__main__":
    global main_thread
    mavros.set_namespace("/mavros")
    rospy.init_node('MAVROS_Listener')

    # main()
    main_thread = threading.Thread(target = main)
    # main_thread.start()
    # main_thread.run()
    app = App().run()
