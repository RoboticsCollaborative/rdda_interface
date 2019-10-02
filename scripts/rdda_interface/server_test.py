#!/usr/bin/env python

import sys
import rospy
from rdda_interface.srv import *

def set_stiffness_client(stiffness):
    rospy.wait_for_service('/rdda_interface/set_stiff')
    try:
        set_stiff = rospy.ServiceProxy('/rdda_interface/set_stiff', SetStiffness)
        res = set_stiff(stiffness)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    stiff = [0,0]
    if len(sys.argv) == 3:
        stiff[0] = float(sys.argv[1])
        stiff[1] = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Set stiffness [%s %s]"%(stiff[0], stiff[1])
    set_stiffness_client(stiff)