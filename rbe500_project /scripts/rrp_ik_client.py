#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from rbe500_project.srv import *

def inverse_kinematics_client(x, y, z):
    rospy.wait_for_service('inverse_kinematics')
    try:
        inverse_kinematics = rospy.ServiceProxy('inverse_kinematics', rrpIK)
        resp = inverse_kinematics(x, y, z)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

print(len(sys.argv))

print("arg 1 ",sys.argv[1])

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        print("We are here!")
    else:
        print(usage())
        sys.exit(1)
        # print("Requesting %s"%(req))
        # # print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
        # print(" Angle are %s "%(inverse_kinematics_client(req)))

    angles = inverse_kinematics_client(x, y, z)
    print(angles)