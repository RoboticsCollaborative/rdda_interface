#include "../../include/rdda_interface/rdda_node.h"

using namespace std;

/* RDDNode constructor */
    RDDNode::RDDNode(ros::NodeHandle &node, Rdda *rddaptr) {
    nh_ = node;
    rdda = rddaptr;
    /* Comment out for remote test */
    /*
    rdda_joint_sub = nh_.subscribe("/rdd/joint_cmds", 1, &RDDNode::subJointCommands_callback, this);
    */
    rdda_joint_pub = nh_.advertise<rdda_interface::JointStates>("/rdd/joint_stats", 1);
}

RDDNode::~RDDNode() {};

/* Publish joint state */
void RDDNode::pubJointStates() {
    rdda_interface::JointStates JointStates_msg;
    JointStates_msg.act_pos.resize(7);

    mutex_lock(&rdda->mutex);
    /*
    if (shared_out->chk == 1) {
    	JointStates_msg.header.frame_id = "time_frame";
    	JointStates_msg.header.stamp.sec = shared_out->timestamp.sec;
    	JointStates_msg.header.stamp.nsec = shared_out->timestamp.nsec;
    	JointStates_msg.act_pos[0] = shared_out->act_pos;
    }
    */
    JointStates_msg.header.frame_id = "time_frame";
    JointStates_msg.header.stamp.sec = rdda->ts.sec;
    JointStates_msg.header.stamp.nsec = rdda->ts.nsec;
    JointStates_msg.act_pos[0] = rdda->motor->motorIn.act_pos;
    mutex_unlock(&rdda->mutex);

    ROS_INFO("Publish joint states [position]: %lf", JointStates_msg.act_pos[0]);
    rdda_joint_pub.publish(JointStates_msg);
}

/* Subscriber callback */
/* Comment out callback for remote test */
/*
void RDDNode::subJointCommands_callback(const rdda_interface::JointCommands::ConstPtr& msg) {

    ticket_lock(&shared_in->queue);
    shared_in->tg_pos = msg->tg_pos[0];
    ticket_unlock(&shared_in->queue);

    ROS_INFO("set target position: %lf", msg->tg_pos[0]);
}
*/

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

    /* Local variables */
    void *p;	/* Intermediate pointer */
    // int err; 	/* Error number */
    int i;	 	/* Loop iterations */
    double pos = 0.0; 

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
