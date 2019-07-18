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
        self.act_pos = [0.0, 0.0]
        self.act_vel = [0.0, 0.0]
        self.ext_tau = [0.0, 0.0]
        self.cmd_tau = [0.0, 0.0]
        self.ts_nsec = 0.0
        self.ts_sec = 0.0

        self.joint_upper_bounds = [0.0, 0.0]
        self.joint_lower_bounds = [0.0, 0.0]
        self.joint_origins = [0.0, 0.0]

    def subjointstates_callback(self, msg):
        self.has_msg = True
        self.act_pos = msg.act_pos
        self.act_vel = msg.act_vel
        self.cmd_tau = msg.cmd_tau
        self.ext_tau = msg.ext_tau
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
            tau_measured = self.ext_tau

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

    """Each finger detect two hardstops then go back to origin"""
    def homing_cmd_tau(self):

        """Make sure ROS message received."""
        while not self.has_msg:
            rospy.sleep(0.01)

        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([10, 10])
        pos_measured = np.array([0.0, 0.0])
        cmd_tau_measured = np.array([0.0, 0.0])
        rate = rospy.Rate(500)
        cmd_tau_threshold = 0.5
        check_upper_bounds = [False, False]
        check_lower_bounds = [False, False]
        check_origins = [False, False]
        time_interval = 2.0e-3
        loop_num = 0

        """Step1: Record the lower bounds of both fingers."""
        while not rospy.is_shutdown() and not (check_lower_bounds[0] and check_lower_bounds[1]):
            loop_num += 1
            pos_measured = self.act_pos
            if loop_num > 200:
                cmd_tau_measured = self.cmd_tau
            for i in range(2):
                if not check_lower_bounds[i]:
                    if np.absolute(cmd_tau_measured[i]) > cmd_tau_threshold:
                        self.joint_lower_bounds[i] = pos_measured[i]
                        check_lower_bounds[i] = True
                    else:
                        pos_ref[i] += -1.0 * time_interval
            self.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)
            rospy.loginfo("cmd_tau_measured_0: {}, cmd_tau_measured_1: {}, ref_pos_0: {}, ref_pos_1: {}"
                          .format(cmd_tau_measured[0], cmd_tau_measured[1], pos_ref[0], pos_ref[1]))
            rate.sleep()
        time.sleep(0.01)

        """Step2: Record upper bound of each finger."""
        cmd_tau_measured = [0.0, 0.0]
        for i in range(2):
            loop_num = 0
            while not rospy.is_shutdown() and not check_upper_bounds[i]:
                loop_num += 1
                pos_measured = self.act_pos
                if loop_num > 200:
                    cmd_tau_measured = self.cmd_tau
                if np.absolute(cmd_tau_measured[i]) > cmd_tau_threshold:
                    self.joint_upper_bounds[i] = pos_measured[i]
                    check_upper_bounds[i] = True
                else:
                    pos_ref[i] += time_interval
                self.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)
                rate.sleep()
            pos_ref[i] = self.joint_lower_bounds[i]
            self.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)
            rospy.loginfo("cmd_tau_measured_0: {}, cmd_tau_measured_1: {}, ref_pos_0: {}, ref_pos_1: {}"
                          .format(cmd_tau_measured[0], cmd_tau_measured[1], pos_ref[0], pos_ref[1]))
            rate.sleep()
        time.sleep(1.5)

        """Step3: Close fingers then record origin when colliding."""
        cmd_tau_measured = [0.0, 0.0]
        loop_num = 0
        while not rospy.is_shutdown() and not (check_origins[0] and check_origins[1]):
            loop_num += 1
            pos_measured = self.act_pos
            if loop_num > 200:
                cmd_tau_measured = self.cmd_tau
            for i in range(2):
                if not check_origins[i]:
                    if np.absolute(cmd_tau_measured[i]) > cmd_tau_threshold:
                        self.joint_origins[i] = pos_measured[i]
                        check_origins[i] = True
                    else:
                        pos_ref[i] += time_interval
            self.publish_joint_cmds(pos_ref=pos_ref, stiffness=stiffness)
            rospy.loginfo("cmd_tau_measured_0: {}, cmd_tau_measured_1: {}, ref_pos_0: {}, ref_pos_1: {}"
                          .format(cmd_tau_measured[0], cmd_tau_measured[1], pos_ref[0], pos_ref[1]))
            rate.sleep()
        time.sleep(0.1)

    """Sinusoid wave for position tests on finger 0. 
        Finger will start at current position, make sure enough space to move."""
    def harmonic_wave(self):
        pos_ref = np.array([0.0, 0.0])
        stiffness = np.array([5.0, 5.0])
        vel_sat = (5.0, 5.0)
        rate = rospy.Rate(500)
        time_interval = 0.0

        while not rospy.is_shutdown():
            time_interval += 2.0e-3
            pos_ref[0] = -0.5 * np.sin(time_interval)
            self.publish_joint_cmds(pos_ref=pos_ref, vel_sat=vel_sat, stiffness=stiffness)
            rospy.loginfo("pos_ref[0]: {}".format(pos_ref[0]))
            rate.sleep()
