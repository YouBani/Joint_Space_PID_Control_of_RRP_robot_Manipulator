#!/usr/bin/env python
from __future__ import print_function
from rbe500_project.srv import rrpIK, rrpIKResponse

import numpy as np
from math import pi, cos, sin, atan2, acos, sqrt
import rospy

def inverse_kinematics(req):
# input: the position of end effector [x, y, z]
# output: joint angles [joint1, joint2, joint3]
    l1 = 0.425
    l2 = 0.345
    L = 1

    # Px = float(position[0])
    # Py = float(position[1])
    # Pz = float(position[2])
    Px = float(req.x)
    Py = float(req.y)
    Pz = float(req.z)

    d_3 = L - Pz

    ctheta2 = (Px**2 + Py**2 - l1**2 - l2**2) / (2 * l1 * l2)  
    stheta2 = math.sqrt(1-math.pow(ctheta2, 2))  

    joint2 = atan2(stheta2, ctheta2)
    
    # ctheta1  =  (px  *  (l1  +  l2  *  ctheta2)  +  py  *  l2  * stheta2) / (px**2 + py**2)  

    alpha = atan2(Py, Px)
    beta = (l1 + l2 * cos(joint2))/sqrt(Px**2 + Py**2)

    joint1 = alpha - beta
    result.theta1 = joint1
    result.theta2 = joint2
    result.d3 = d_3
    # return [joint1, joint2, joint3]
    return rrpIKResponse(result)

def inverse_kinematics_server():
    rospy.init_node('rrp_ik_server')
    # s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    s = rospy.Service('inverse_kinematics', rrpIK , inverse_kinematics) 
    print("Ready to calculate the inverse kinematics")
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()