import rospy
from rdda_interface.msg import JointCommands
from rdda_interface.msg import JointStates


class RddaProxy:

    def __init__(self):
        self.joint_pub = rospy.Publisher("rdd/joint_cmds", JointCommands, queue_size=1)
        self.joint_sub = rospy.Subscriber("rdd/joint_stats", JointStates, self.subjointstates_callback)

        self.act_pos = None
        self.act_vel = None

    def subjointstates_callback(self, msg):
        self.act_pos = msg.data.act_pos
        self.act_vel = msg.data.act_vel

    def publish_joint_cmds(self, joint_commands):
        joint_cmd_msg = JointCommands()
        joint_cmd_msg.pos_ref = joint_commands[0]
        joint_cmd_msg.stiffness = joint_commands[1]

        self.joint_pub.publish(joint_cmd_msg)


