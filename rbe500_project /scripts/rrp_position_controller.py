#!/usr/bin/env python
import rospy
import csv
import time
from sensor_msgs.srv import *
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64
from rbe500_project.srv import rrpIK, rrpIKResponse
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from rrp_ik_server import inverse_kinematics


# 100Hz frequency
ros_rate = 100.0

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
    curr_point = []
    timer_started = False


def call_back(data):
    global curr_theta1, curr_theta2, curr_d3
    curr_theta1 = data.position[0]
    curr_theta2 = data.position[1]
    curr_d3 = data.position[2]
    print(curr_theta1)


def get_position():
    rate_value = 100
    rospy.init_node("listner", anonymous=True)
    joint_call = rospy.Subscriber("/rrp/joint_states", JointState, call_back)
    rate = rospy.Rate(rate_value)
    rate.sleep()


def joint_effort(effort, jointN):
    rate_value = 100
    # rospy.init_node('joint_ctrl', anonymous=True)
    rate = rospy.Rate(rate_value) # 100h
    pub = rospy.Publisher('/rrp/'+jointN+'_effort_controller/command', Float64, queue_size=1)

    # while not rospy.is_shutdown():
    # while count <100:
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
    totalErrorJ1 = 0
    totalErrorJ2 = 0
    totalErrorJ3 = 0
    print("position_err", position_err)
    # while position_err > 0.10:
    if joint_name is 'theta1':

        print(position_err)
        derivative_err = (last_positions[0] - curr_point)/sampling_rate
        totalErrorJ1 = (totalErrorJ1 + position_err)*sampling_rate

        # Set reference vals
        last_positions[0] = curr_point
        last_set_points[0] = set_point

        # PID output
        effort = position_err*Kp_vals[0] + totalErrorJ1*Ki_vals[0] + derivative_err*Kd_vals[0]
        # print("effort loop: ", effort)

    elif joint_name is 'theta2':

        derivative_err = (last_positions[1] - curr_point)/sampling_rate
        totalErrorJ2 = (totalErrorJ2 + position_err)*sampling_rate
        # Set reference vals
        last_positions[1] = curr_point
        last_set_points[1] = set_point

        # PID output
        effort = position_err*Kp_vals[1] + totalErrorJ2*Ki_vals[1] + derivative_err*Kd_vals[1]
    elif joint_name is 'd3':

        derivative_err = (last_positions[2] - curr_point)/sampling_rate
        totalErrorJ3 = (totalErrorJ3 + position_err)*sampling_rate

        # Set reference vals
        last_positions[2] = curr_point
        last_set_points[2] = set_point

        # PID output
        effort = position_err*Kp_vals[2] + totalErrorJ3*Ki_vals[2] + derivative_err*Kd_vals[2]
        # Append data entries to lists
        times.append(time.time())
        set_points.append(set_point)
        curr_points.append(curr_point)

    # print("effort", effort)

    if joint_name == "theta1":
        joint_effort(effort, "joint1")
    elif joint_name == "theta2":
        joint_effort(effort, "joint2")
    elif joint_name == "d3":
        joint_effort(effort, "joint3")


if __name__ == "__main__":
    # works
    # Kp_vals = [5, 3, 450]
    # Ki_vals = [0.01, 0.01, 0.01]
    # Kd_vals = [17, 5, 0.05]

    # Kp_vals = [5, 3, 450]
    # Ki_vals = [0.01, 0.1, 0.01]
    # Kd_vals = [17, 6, 0.05]

    # Kp_vals = [12, 4, 450]
    # Ki_vals = [0.001, 0.001, 0.01]
    # Kd_vals = [14, 10, 0.05]

    Kp_vals = [7, 3, 450]
    Ki_vals = [0.01, 0.01, 0.01]
    Kd_vals = [17, 5, 0.05]

    case1 = [0, 0.77, 0.34]
    case2 = [-0.345, 0.425, 0.24]
    case3 = [-0.67, -0.245, 0.14]
    case4 = [0.77, 0.0, 0.39]
    test_cases = [case1, case2, case3, case4]
    # test_cases = [case3, case4]
    # test_cases = [case2]
    for test_case in test_cases:
        joint_angle = inverse_kinematics(test_case)
        try:
            get_position()
            while abs(joint_angle.joint_vals.theta1 - curr_theta1) > 0.15:
                # print("the error", joint_angle.joint_vals.theta1 - curr_theta1)
                control(joint_angle.joint_vals.theta1, curr_theta1, "theta1")
                # joint_effort(0, "joint2")
            joint_effort(0, "joint1")

            while abs(joint_angle.joint_vals.theta2 - curr_theta2) > 0.15:
                # joint_effort(0, "joint1")
                control(joint_angle.joint_vals.theta2, curr_theta2, "theta2")
                joint_effort(0, "joint1")
            joint_effort(0, "joint2")

            while joint_angle.joint_vals.d3 - curr_d3 > 0.002:
                control(joint_angle.joint_vals.d3, curr_d3, "d3")
            joint_effort(0, "joint3")
            rospy.sleep(1)

        except rospy.ROSInterruptException:
            pass
