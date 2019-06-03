#include "../../include/rdda_interface/rdda_node.h"

using namespace std;

/* RDDNode constructor */
    RDDNode::RDDNode(ros::NodeHandle &node, Rdda *rddaptr) {
    nh_ = node;
    rdda = rddaptr;
    /* Comment out for remote test */
    rdda_joint_sub = nh_.subscribe("/rdd/joint_cmds", 1, &RDDNode::subJointCommands_callback, this);
    rdda_joint_pub = nh_.advertise<rdda_interface::JointStates>("/rdd/joint_stats", 1);
}

RDDNode::~RDDNode() {};

/* Publish joint state */
void RDDNode::pubJointStates() {
    rdda_interface::JointStates JointStates_msg;
    JointStates_msg.act_pos.resize(7);
    JointStates_msg.act_vel.resize(7);

    mutex_lock(&rdda->mutex);
    /*
    JointStates_msg.header.frame_id = "time_frame";
    JointStates_msg.header.stamp.sec = rdda->ts.sec;
    JointStates_msg.header.stamp.nsec = rdda->ts.nsec;
    JointStates_msg.act_pos[0] = rdda->motor[0].motorIn.act_pos;
    JointStates_msg.act_pos[1] = rdda->motor[1].motorIn.act_pos;
    */
    for (int i=0; i<2; ++i) {
 	JointStates_msg.header.frame_id = "time_frame";
    	JointStates_msg.header.stamp.sec = rdda->ts.sec;
    	JointStates_msg.header.stamp.nsec = rdda->ts.nsec;
    	JointStates_msg.act_pos[i] = rdda->motor[i].motorIn.act_pos;
    	JointStates_msg.act_vel[i] = rdda->motor[i].motorIn.act_vel;
    }
    mutex_unlock(&rdda->mutex);

    ROS_INFO("Publish joint states [position]: %lf", JointStates_msg.act_pos[0]);
    //ROS_INFO("ROS interface testing...");
    rdda_joint_pub.publish(JointStates_msg);
}

/* Subscriber callback */
/* Comment out callback for remote test */
void RDDNode::subJointCommands_callback(const rdda_interface::JointCommands::ConstPtr& msg) {

    mutex_lock(&rdda->mutex);
    /*
    for (int i=0; i<2; ++i) {
        rdda->motor[i].motorOut.vel_off = msg->vel_sat[i];
    }
    mutex_unlock(&rdda->mutex);
    */

    ROS_INFO("set vel_sat[0]: %lf", msg->vel_sat[0]);
}

/* Run loop */
void RDDNode::run() {
    ros::Rate loop_rate(500);
    while (ros::ok()) {
	/* Publisher (wrap) */
	pubJointStates();
	/* Subscriber callback loop */
	ros::spinOnce();
	loop_rate.sleep();
    }
}

int main(int argc, char** argv) {

    /* Instanciate input-output data varibles */
    Rdda *rdda;

    /* Map data structs to shared memory */
    /* Open and obtain shared memory pointers for master-input data */
    rdda = initRdda();
    if (rdda == NULL) {
        fprintf(stderr, "Init rdda failed.\n");
        exit(1);
    }

    /* Initialise ROS node */
    ros::init(argc, argv, "rdd_node");
    printf("Launch ros interface\n");

    ros::NodeHandle node("~");
    RDDNode rdd(node, rdda);
    rdd.run();
}
