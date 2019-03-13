#!/usr/bin/env python
import rospy
from rdda_interface.msg import JointCommands
from rdda_interface.msg import JointStates

class RddProxy:
    def __init__(self):
	self.joint_commands = None
	self.joint_states = None

	self.joint_pub = rospy.Publisher("rdd/joint_cmds", JointCommands, queue_size = 1)
	self.joint_sub = rospy.Subscriber("rdd/joint_stats", JointStates, self.subJointStates_callback)

    def subJointStates_callBack(self, msg):
	rospy.loginfo(rospy.get_caller_id() + "Joint States: %lf", msg.act_pos[0])
