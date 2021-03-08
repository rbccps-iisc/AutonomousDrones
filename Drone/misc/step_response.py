#!/usr/bin/python

import rospy, time
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import SetMode
#from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header

gazebo_x_vel = mavros_e_vel = commanded_e_vel = 0.0
plot_arrays = [[],[],[],[]]
start_timer = time.time()
time_elapsed = 0


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


def gazebo_cb(msg):
	global gazebo_x_vel

	#APM
	# gazebo_x_vel = -msg.twist[0].linear.y

	#PX4
	gazebo_x_vel = msg.twist[2].linear.x


def mavros_vel_cb(msg):
	global mavros_e_vel

	mavros_e_vel = msg.twist.linear.x


def comm_vel_cb(msg):
	global commanded_e_vel

	commanded_e_vel = msg.twist.linear.x

rospy.init_node('step_response')

rate = rospy.Rate(20)

#rospy.Subscriber('/gazebo/model_states', ModelStates, gazebo_cb)
rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, mavros_vel_cb)
rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel', TwistStamped, comm_vel_cb)

pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

for i in range(0, 50):
    pub.publish(twist_obj(0, 0, 0, 0.0, 0.0, 0.0))
    time.sleep(0.01)

set_mode(0, 'OFFBOARD')

# time.sleep(1)
# while commanded_e_vel == -1.5: print("Wait")

while(not rospy.is_shutdown() and time_elapsed < 9.0):
# while(not rospy.is_shutdown()):
	time_elapsed = time.time() - start_timer

	if(time_elapsed < 1.0):
		pub.publish(twist_obj(0, 0, 0, 0, 0, 0))

		plot_arrays[0].append(0)
		# plot_arrays[0].append(commanded_e_vel)
#		plot_arrays[1].append(gazebo_x_vel)
		plot_arrays[2].append(mavros_e_vel)
		plot_arrays[3].append(time_elapsed)

	elif(time_elapsed >= 1.0 and time_elapsed <= 5.0):
		pub.publish(twist_obj(1, 0, 0, 0, 0, 0))

		plot_arrays[0].append(1)
		# plot_arrays[0].append(commanded_e_vel)
#		plot_arrays[1].append(gazebo_x_vel)
		plot_arrays[2].append(mavros_e_vel)
		plot_arrays[3].append(time_elapsed)

	else:
		pub.publish(twist_obj(0, 0, 0, 0, 0, 0))

		plot_arrays[0].append(0)
		# plot_arrays[0].append(commanded_e_vel)
#		plot_arrays[1].append(gazebo_x_vel)
		plot_arrays[2].append(mavros_e_vel)
		plot_arrays[3].append(time_elapsed)

	print(mavros_e_vel)
	rate.sleep()

plt.plot(plot_arrays[3], plot_arrays[0], plot_arrays[3], plot_arrays[2])  #ot_arrays[1])
plt.xlabel('Time in s')
plt.ylabel('Velocity in m/s')
plt.ylim(-1.5, 1.5)

# plt.legend()
# plt.plot(plot_arrays[3], plot_arrays[0])
print(len(plot_arrays[0]), len(plot_arrays[1]), len(plot_arrays[2]), len(plot_arrays[3]))
plt.show()
