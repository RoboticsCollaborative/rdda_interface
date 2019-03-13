#!/usr/bin/env python
import rospy
import math
from rdda_interface.msg import JointCommands
from rdda_interface.msg import JointStates
#from RddProxy import RddProxy

#master = RddProxy()

#def subJointStates_callback(data):
#    rospy.loginfo("Actual position: %lf", data.act_pos[0])

i = 0

"""
def pubJointCommands():
    msg = JointCommands()
    if len(msg.tg_pos):
	msg.tg_pos[0] = 3.14159
    else:
	msg.tg_pos.append(3.14159)
    joint_pub.publish(msg)
    rospy.loginfo("Actual position: %lf", msg.tg_pos[0])
"""

def run():
    global i
    msg = JointCommands()
    while not rospy.is_shutdown():
	pos = math.cos(2*3.14159/1000 * i)
        if len(msg.tg_pos):
   	    msg.tg_pos[0] = pos
    	else:
	    msg.tg_pos.append(pos)
	i += 1
    	joint_pub.publish(msg)
    	rospy.loginfo("Actual position: %lf", msg.tg_pos[0])
#	rospy.spin()
	rate.sleep()

if __name__ == '__main__':
    try:
    	joint_pub = rospy.Publisher("rdd/joint_cmds", JointCommands, queue_size = 1)
    	rospy.init_node('ros_master', anonymous=True)
    	rate = rospy.Rate(1000)
#	rospy.Subscriber("rdd/joint_stats", JointStates, subJointStates_callback)
	run()
    except rospy.ROSInterruptException:
	pass
