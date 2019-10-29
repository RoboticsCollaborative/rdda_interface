#!/usr/bin/env python

import rospy
from RddaProxy import RddaProxy


def main():
    rdda = RddaProxy()
    rospy.init_node('homing', anonymous=True)

    rdda.homing()

    # rospy.set_param('/rdda_interface/origins', rdda.joint_origins)
    # rospy.set_param('/rdda_interface/upper_bounds', rdda.joint_upper_bounds)
    # rospy.set_param('/rdda_interface/lower_bounds', rdda.joint_lower_bounds)
    rospy.set_param('/origins', rdda.joint_origins)
    rospy.set_param('/upper_bounds', rdda.joint_upper_bounds)
    rospy.set_param('/lower_bounds', rdda.joint_lower_bounds)
    rospy.set_param('/has_home', 1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
