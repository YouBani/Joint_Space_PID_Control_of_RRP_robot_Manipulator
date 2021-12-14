#!/usr/bin/env python
import rospy
import csv
import time
# from gazebo_msgs.srv import *
from sensor_msgs.srv import *
from sensor_msgs.msg import JointState

# from rbe500_project.msg import joint_angles 
from std_msgs.msg import Float64
from rbe500_project.srv import rrpIK, rrpIKResponse
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from rrp_ik_server import inverse_kinematics



joint_names = ['theta1', 'theta2', 'd3']
case1 = [0, 0.77, 0.34]
case2 = [-0.345, 0.425, 0.24]
case3 = [-0.67, -0.245, 0.14]
case4 = [0.77, 0.0, 0.39]
case5 = [0.425, 0.344, 0.29]
test_case = case2
joint_angle = inverse_kinematics(test_case)
# print(joint_angle.joint_vals.theta1)
# print(joint_angle["joint_vals"])


joint_names = 'theta1', 'theta2', 'd3'

Kp_vals = [25, 70, 40]
Kd_vals = [25, 10, 20]

# 10Hz frequency
ros_rate = 10.0

# Loop execution rate in seconds (1/freq)
sampling_rate = (1/ros_rate) 
last_positions = [0.0, 0.0, 0.0]
last_set_points = [0.0, 0.0, 0.0]

# Record data every 15 seconds
record_time_interval = 15

start_time = 0
timer_started = False
times = []
set_points = []
curr_points = []

def reset_timer():
    global times, set_points, curr_points, timer_started

    times = []
    set_points = []
    curr_points = []
    timer_started = False

curr_theta1 = 0
curr_theta2 = 0
curr_d3 = 0

def call_back(data):
    global curr_theta1, curr_theta2, curr_d3
    curr_theta1 = data.position[0]
    curr_theta2 = data.position[1]
    curr_d3 = data.position[2]

    # control(joint_angle.joint_vals.theta1, curr_theta1, joint_names[0])
    # control(joint_angle.joint_vals.theta2, curr_theta2, joint_names[1])
    # control(joint_angle.joint_vals.d3, curr_d3, joint_names[2])


def get_position():
    rospy.init_node("listner", anonymous=True)
    
    # joint_call = rospy.Subscriber("/rrp/joint_states", JointState, joint_state_callback)
    joint_call = rospy.Subscriber("/rrp/joint_states", JointState, call_back)
    # joint_data = joint_call(joint)
    # rospy.Rate(10)
    rospy.spin()
    # position = joint_state.position
    # print("theta1, theta2, d3: ", position)
    # return position
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Action terminated.")



def joint_effort(effort, jointN):
    rate_value = 10
    rospy.init_node('joint_ctrl', anonymous=True)
    rate = rospy.Rate(rate_value) # 10h
    pub = rospy.Publisher('/rrp/'+jointN+'_effort_controller/command', Float64, queue_size=1)
    while not rospy.is_shutdown():
        try:  
            pub.publish(effort)
            rate.sleep()
        except rospy.ROSInterruptException:
	        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
	        rospy.logerr("ROS Time Backwards! Just ignore the exception!") 




def control(set_point, curr_point, joint_name):
    global last_positions, last_set_points, timer_started, times, set_points, curr_points, start_time

    # Start timer for recording data
    if not timer_started:
        start_time = time.time()
        timer_started = True

    # Calculate errors
    position_err = set_point - curr_point

    if (joint_name is 'theta1'):
        

        derivative_err = (last_positions[0] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[0] = curr_point
        last_set_points[0] = set_point

        # PD output
        effort = position_err*Kp_vals[0] + derivative_err*Kd_vals[0]
        
    elif (joint_name is 'theta2'):

        derivative_err = (last_positions[1] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[1] = curr_point
        last_set_points[1] = set_point

        # PD output
        effort = position_err*Kp_vals[1] + derivative_err*Kd_vals[1]
    elif (joint_name is 'd3'):

        derivative_err = (last_positions[2] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[2] = curr_point
        last_set_points[2] = set_point

        # PD output
        effort = position_err*Kp_vals[2] + derivative_err*Kd_vals[2]

    # Append data entries to lists
    times.append(time.time())
    set_points.append(set_point)
    curr_points.append(curr_point)
    print("effort", effort)

    if joint_name == "theta1":
        joint_effort(effort, "joint1")
    elif joint_name == "theta2":
        joint_effort(effort, "joint2")
    elif joint_name == "d3":
        joint_effort(effort, "joint3")


def joint_state_callback(msg):
    joint_state = JointState()
    joint_state_position = msg.position
    joint_state_velocity = msg.velocity
    joint_state_name = msg.name

    theta1 = joint_state_position[0]
    theta2 = joint_state_position[1]
    d3 = joint_state_position[2]
    print(theta1, theta2, d3)
    # do_pd_control(set_point, curr_point, joint_name):
    control(joint_angle.joint_vals.theta1, theta1, joint_names[0])
    control(joint_angle.joint_vals.theta2, theta2, joint_names[1])
    control(joint_angle.joint_vals.d3, d3, joint_names[2])


if __name__ == "__main__":
    try:
        # get_position()
        # print(curr_theta1, curr_theta2, curr_d3)
        control(joint_angle.joint_vals.theta1, curr_theta1, "theta1")
        # joint_effort(2, "joint1")
        # theta1 = get_position()[0]
        # theta1 = joint_state_position[0]
        # theta2 = joint_state_position[1]
        # d3 = joint_state_position[2]
        # control(last_set_points[0], get_position(joint_names[0]), joint_names[0])
    # do_pd_control(set_point, curr_point, joint_name):
        # control(joint_angle.joint_vals.theta1, theta1, joint_names[0])
        # get_position()
        # control(last_set_points[0], get_position()[0], joint_names[0]) 
        # control(last_set_points[0], get_position(joint_names[0]), joint_names[0])
    except rospy.ROSInterruptException:
        pass