import rospy
from rdda_interface.msg import JointCommands
from rdda_interface.msg import JointStates

import numpy as np


class RddaProxy:

    def __init__(self):
        self.joint_pub = rospy.Publisher("rdd/joint_cmds", JointCommands, queue_size=1)
        # self.joint_sub = rospy.Subscriber("rdd/joint_stats", JointStates, self.subjointstates_callback)

        """Joint states"""
        self.act_pos = None
        self.act_vel = None
        self.act_tau = None
        self.ts_nsec = None
        self.ts_sec = None

        """Joint commands"""
        self.pos_ref = np.array([0.0, 0.0])
        self.vel_sat = np.array([3.0, 3.0])
        self.tau_sat = np.array([5.0, 5.0])
        self.stiffness = np.array([0.1, 0.1])
        self.freq_anti_alias = 500.0

    # def subjointstates_callback(self, msg):
    #     self.act_pos = msg.data.act_pos
    #     self.act_vel = msg.data.act_vel
    #     self.act_tau = msg.data.act_tau
    #     self.ts_nsec = msg.data.ts_nsec
    #     self.ts_sec = msg.data.ts_sec

    def publish_joint_cmds(self):
        joint_cmd_msg = JointCommands()
        joint_cmd_msg.pos_ref = self.pos_ref
        joint_cmd_msg.vel_sat = self.vel_sat
        joint_cmd_msg.tau_sat = self.tau_sat
        joint_cmd_msg.stiffness = self.stiffness
        joint_cmd_msg.freq_anti_alias = self.freq_anti_alias

        self.joint_pub.publish(joint_cmd_msg)


