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
    l3 = 0.11

    # Px = float(position[0])
    # Py = float(position[1])
    # Pz = float(position[2])
    Px = req.x
    Py = req.y
    Pz = req.z
    # Px = 0.44
    # d_3 = l3 - Pz
    d3 = Pz
    print("Px", Px)
    print("Py", Py)
    print("d3", d3)
    ctheta2 = (Px**2 + Py**2 - l1**2 - l2**2) / (2 * l1 * l2)  
    print(ctheta2)
    stheta2 = sqrt(1- (ctheta2 ** 2))

    theta2 = atan2(stheta2, ctheta2)
    
    # ctheta1  =  (px  *  (l1  +  l2  *  ctheta2)  +  py  *  l2  * stheta2) / (px**2 + py**2)  

    alpha = atan2(Py, Px)
    beta = (l1 + l2 * cos(theta2))/sqrt(Px**2 + Py**2)

    tetha1 = alpha - beta
    # result.theta1 = joint1
    # result.theta2 = theta2
    # result.d3 = d3
    print(tetha1)
    print(theta2)
    print(d3)

    # return [joint1, joint2, joint3]
    # return rrpIKResponse(result)

def inverse_kinematics_server():
    rospy.init_node('rrp_ik_server')
    s = rospy.Service('inverse_kinematics', rrpIK , inverse_kinematics) 
    print("Ready to calculate the inverse kinematics")
    rospy.spin()

if __name__ == "__main__":
    inverse_kinematics_server()