import numpy as np
from math import pi, cos, sin, atan2, acos, sqrt
import rospy

def inverse_kinematics(position):
# input: the position of end effector [x, y, z]
# output: joint angles [joint1, joint2, joint3]

    x = position[0]
    y = position[1]
    z = position[2]
    joint3 = z


    return [joint1, joint2, joint3]

def inverse_kinematics_server():
    rospy.init_node('rrp_ik_server')
    # s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    s = rospy.Service('inverse_kinematics', inverseKimematics , inverse_kinematics) 
    print("Ready to calculate the inverse kinematics")
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()