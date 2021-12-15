#!/usr/bin/env python
from __future__ import print_function

from rbe500_project.srv import rrpIK, rrpIKResponse
from rbe500_project.msg import joint_angles

import numpy as np
from math import pi, cos, sin, atan2, acos, sqrt
import rospy
from geometry_msgs.msg import Vector3

result = joint_angles()

def inverse_kinematics(req):
# input: the position of end effector [x, y, z]
# output: joint angles [joint1, joint2, joint3]

    # angles = Vector3()
    l = 0.45
    l1 = 0.425
    l2 = 0.345
    l3 = 0.345 + 0.05

    # For using the inverse kinematic client 
    Px = float(req[0])
    Py = float(req[1])
    Pz = float(req[2])
    # For using the p1, p2, p3 and p4 lists 
    # Px = req.x
    # Py = req.y
    # Pz = req.z

    if Py==0:
        if Px>0:
            theta1 = 0
            theta2 = 0
        else:
            theta2 = 0
            theta1 = pi
    elif Px == 0:
        if Py>0:
            theta1 = pi/2
            theta2 = 0
        else:
            theta2 = 0
            theta1 = -pi/2
    else:
        theta2 = acos((Px**2 + Py**2 - l1**2 - l2**2)/(2*l1*l2))
        theta1 = atan2(Py, Px) - atan2((l2+l1)*sin(theta2), l1+l2+cos(theta2))
        theta1 = abs(theta1)
    d3 = l3 - Pz

    result.theta1 = theta1  
    result.theta2 = theta2
    result.d3 = d3
    return rrpIKResponse(result)


def inverse_kinematics_server():
    rospy.init_node('rrp_ik_server')
    # s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    s = rospy.Service('inverse_kinematics', rrpIK , inverse_kinematics) 
    print("Ready to calculate the inverse kinematics")
    rospy.spin()


if __name__ == "__main__":
    inverse_kinematics_server()
