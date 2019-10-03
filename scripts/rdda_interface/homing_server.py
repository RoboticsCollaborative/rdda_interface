#!/usr/bin/env python

import rospy

from rdda_interface.srv import Homing, HomingResponse

from RddaProxy import RddaProxy


def handle_homing():
    rdda = RddaProxy()

    """ Homing routine. """
    rdda.homing()

    origins = rdda.joint_origins
    return HomingResponse(origins)


def homing_server():
    rospy.init_node('homing_server', anonymous=True)
    rospy.Service('/rdda_interface/homing', Homing, handle_homing)
    rospy.spin()


if __name__ == "__main__":
    homing_server()
