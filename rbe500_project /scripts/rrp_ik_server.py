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


    # Px = float(position[0])
    # Py = float(position[1])
    # Pz = float(position[2])
    # Px = req.x
    # Py = req.y
    # Pz = req.z
    Px = float(req[0])
    Py = float(req[1])
    Pz = float(req[2])
    # Px = 0.44
    # d3 = l3 - Pz
    
    theta2 = acos((Px**2 + Py**2 - l1**2 - l2**2)/(2*l1*l2))
    # C_theta2 = cos((Px**2 + Py**2 - l1**2 - l2**2)/(2*l1*l2))
    # S_theta2 = sqrt(1 - C_theta2**2)
    # theta2 = atan2(S_theta2,C_theta2)
    theta1 = atan2(Py, Px) - atan2((l2+l1)*sin(theta2), l1+l2+cos(theta2))
    # theta1 = atan2(Py, Px) - (l1 + l2 * cos(theta2))/sqrt(Px**2 + Py**2)
    d3 = l3 - Pz

#====#
    # d3 = Pz
    # cos_theta2 = acos(Px**2 + Py**2 - l1**2 - l2**2) / (2 * l1 * l2)  
    # sin_theta2 = sqrt(1-(cos_theta2**2))

    # theta2 = atan2(sin_theta2, cos_theta2)
    # print("theta2", theta2)
    # alpha = atan2(Py, Px)
    # beta = (l1 + l2 * cos(theta2))/sqrt(Px**2 + Py**2)

    # # alpha = math.atan2(y,x)
    # # D1 = (x**2+y**2)/(2*L*math.sqrt(x**2+y**2))
    # # C1 = math.sqrt(1-(D1**2))
    # # beta = math.atan2(C1, D1)
    
    # theta1 = alpha - beta

#===
    # stheta2 = sqrt(1 - (ctheta2**2))
    # print("stheta2", stheta2)
    # theta2 = atan2(stheta2, ctheta2)
    # theta2 = acos(ctheta2)
    # print("theta2", theta2)
    
    # ctheta1  =  (px  *  (l1  +  l2  *  ctheta2)  +  py  *  l2  * stheta2) / (px**2 + py**2)  


    # result.theta1 = joint1
    # result.theta2 = theta2
    # result.d3 = d3
    # print(tetha1)
    # print(theta2)
    # print(d3)
    # angles.x = theta1
    # angles.y = theta2
    # angles.z = d3
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