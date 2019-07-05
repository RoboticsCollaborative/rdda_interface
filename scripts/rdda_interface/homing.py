#!/usr/bin/env python
import rospy
import numpy as np
from RddaProxy import RddaProxy
# from rdda_interface.msg import JointCommands


def main():
    rdda = RddaProxy()
    rospy.init_node('homing', anonymous=True)
    # rate = rospy.Rate(500)
    # time_interval = 0.0

    # pos_ref = np.array([0.0, 0.0])
    # stiffness = np.array([5, 5])

    # while not rospy.is_shutdown():
    #     time_interval += 2e-3
    #     pos_ref[0] = -1.0 * np.sin(time_interval)
    #     rdda.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)
    #     rospy.loginfo("pos_ref[0]: {}".format(pos_ref[0]))
    #     rate.sleep()
    rdda.homing()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
