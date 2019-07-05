import rospy
from rdda_interface.msg import JointCommands
from rdda_interface.msg import JointStates

import time
import numpy as np


class RddaProxy:

    def __init__(self):
        self.joint_pub = rospy.Publisher("rdd/joint_cmds", JointCommands, queue_size=1)
        self.joint_sub = rospy.Subscriber("rdd/joint_stats", JointStates, self.subjointstates_callback)

        self.has_msg = False

        """Joint states"""
        self.act_pos = (0.0, 0.0)
        self.act_vel = (0.0, 0.0)
        self.act_tau = (0.0, 0.0)
        self.ts_nsec = 0.0
        self.ts_sec = 0.0

    def subjointstates_callback(self, msg):
        self.has_msg = True
        self.act_pos = msg.act_pos
        self.act_vel = msg.act_vel
        self.act_tau = msg.act_tau
        self.ts_nsec = msg.ts_nsec
        self.ts_sec = msg.ts_sec

    def publish_joint_cmds(self, pos_ref=(0.0, 0.0), vel_sat=(5.0, 5.0),
                           tau_sat=(5.0, 5.0), stiffness=(0.0, 0.0), freq_anti_alias=500.0):

        joint_cmd_msg = JointCommands()
        joint_cmd_msg.pos_ref = pos_ref
        joint_cmd_msg.vel_sat = vel_sat
        joint_cmd_msg.tau_sat = tau_sat
        joint_cmd_msg.stiffness = stiffness
        joint_cmd_msg.freq_anti_alias = freq_anti_alias

        self.joint_pub.publish(joint_cmd_msg)

    """Fingers return to origin with arbitrary initial conditions"""
    def homing(self):

        """Make sure ROS message received"""
        while not self.has_msg:
            rospy.sleep(0.01)

        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([10, 10])
        rate = rospy.Rate(500)
        time_interval = 0.0
        tau_threshold = np.array([0.14, 0.14])
        done_finger0 = False
        done_finger1 = False
        done = False

        while not done and not rospy.is_shutdown():
            time_interval += 2e-6
            # pos_ref[0] = -1.0 * np.sin(time_interval)
            tau_measured = self.act_tau

            if not done_finger0:
                if tau_measured[0] > tau_threshold[0]:
                    pos_ref[0] += 1.27
                    done_finger0 = True
                else:
                    pos_ref[0] += -1.0 * time_interval
            if not done_finger1:
                if tau_measured[1] > tau_threshold[1]:
                    pos_ref[1] += 1.15
                    done_finger1 = True
                else:
                    pos_ref[1] += -1.0 * time_interval

            self.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)

            # rospy.loginfo("pos_ref[0]: {}".format(pos_ref[0]))
            rate.sleep()
            done = done_finger0 and done_finger1

        time.sleep(0.5)

    """Sinusoid wave for position tests on finger 0. 
        Finger will start at current position, make sure enough space to move."""
    def harmonic_wave(self):
        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([5.0, 5.0])
        vel_sat = (3.0, 3.0)
        rate = rospy.Rate(500)
        time_interval = 0.0

        while not rospy.is_shutdown():
            time_interval += 2.0e-3
            pos_ref[0] = -0.5 * np.sin(time_interval)
            self.publish_joint_cmds(pos_ref=pos_ref, vel_sat=vel_sat, stiffness=stiffness)
            rospy.loginfo("pos_ref[0]: {}".format(pos_ref[0]))
            rate.sleep()
