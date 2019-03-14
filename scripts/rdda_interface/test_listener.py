#!/usr/bin/env python
import rospy
from rdda_interface.msg import JointStates


def callback(data):
    rospy.loginfo("Joint States: %lf", data.act_pos[0])


def listener():
    rospy.init_node('test_listener', anonymous=True)
    rospy.Subscriber('/rdd/joint_stats', JointStates, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
