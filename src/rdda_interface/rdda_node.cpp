#include "../../include/rdda_interface/rdda_node.h"

using namespace std;

/* RDDNode constructor */
RDDNode::RDDNode(ros::NodeHandle &node, shared_in_t *in, shared_out_t *out) {
    nh_ = node;
    shared_in = in;
    shared_out = out;
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

    ticket_lock(&shared_out->queue);
    if (shared_out->chk == 1) {
    	JointStates_msg.header.frame_id = "time_frame";
    	JointStates_msg.header.stamp.sec = shared_out->timestamp.sec; /* Timestamp */
    	JointStates_msg.header.stamp.nsec = shared_out->timestamp.nsec;
    	JointStates_msg.act_pos[0] = shared_out->act_pos; /* Publish actual position */
    }
    ticket_unlock(&shared_out->queue);

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
    shared_in_t *shared_in;
    shared_out_t *shared_out;

    /* Map data structs to shared memory */
    /* Open and obtain shared memory pointers for master-input data */
    if (!openSharedMemory(SHARED_IN, &p)) {
	shared_in = (shared_in_t *) p;
    } else {
	fprintf(stderr, "Open(shared_in)\n");
	return -1;
    }
    /* Initialise ticket lock */
    ticket_init(&shared_in->queue);

    /* Open and obtain shared memory pointers for master-output data */
    if (!openSharedMemory(SHARED_OUT, &p)) {
	shared_out = (shared_out_t *) p;
    } else {
	fprintf(stderr, "Open(shared_out)\n");
	return -1;
    }
    /* Initialise ticket lock */
    ticket_init(&shared_out->queue);

    /* initialise memory data*/	
    ticket_lock(&shared_out->queue);
    shared_out->chk = 0;
    shared_out->act_pos = (double)0.0;
    shared_out->timestamp.sec = 0;
    shared_out->timestamp.nsec = 0;
    ticket_unlock(&shared_out->queue);

    /* Initialise ROS node */
    ros::init(argc, argv, "rdd_node");
    printf("Launch ros interface\n");

    ros::NodeHandle node("~");
    RDDNode rdd(node, shared_in, shared_out);
    rdd.run();

}
