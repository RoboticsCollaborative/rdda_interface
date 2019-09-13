#include "../../include/rdda_interface/rdda_node.h"

using namespace std;

/* RDDNode constructor */
    RDDNode::RDDNode(ros::NodeHandle &node, Rdda *rddaptr) {
    nh_ = node;
    rdda = rddaptr;

    rdda_joint_sub = nh_.subscribe("/rdd/joint_cmds", 1, &RDDNode::subJointCommands_callback, this);
//    rdda_joint_pub = nh_.advertise<rdda_interface::JointStates>("/rdd/joint_stats", 1);
    rdda_joint_pub = nh_.advertise<sensor_msgs::JointState>("/rdd/joint_stats", 1);
    rdda_ctrl_pub = nh_.advertise<rdda_interface::ControlState>("/rdd/ctrl_stats", 1);

    rdda_maxvel_srv = nh_.advertiseService("/rdd/set_max_vel", &RDDNode::setMaxVel, this);
    rdda_maxeff_srv = nh_.advertiseService("/rdd/set_max_eff", &RDDNode::setMaxEffort, this);
    rdda_stiff_srv = nh_.advertiseService("/rdd/set_stiff", &RDDNode::setStiffness, this);
}

RDDNode::~RDDNode() {};

/* Publish joint state */
void RDDNode::pubJointStates() {
//    rdda_interface::JointStates JointStates_msg;
//    JointStates_msg.act_pos.resize(2);
//    JointStates_msg.act_vel.resize(2);
//    JointStates_msg.ext_tau.resize(2);
//    JointStates_msg.cmd_tau.resize(2);

    sensor_msgs::JointState JointStates_msg;
//    JointStates_msg.name.resize(2);
    JointStates_msg.position.resize(2);
    JointStates_msg.velocity.resize(2);
    JointStates_msg.effort.resize(2);
    rdda_interface::ControlState ControlStates_msg;
    ControlStates_msg.applied_effort.resize(2);

    mutex_lock(&rdda->mutex);

////    JointStates_msg.header.frame_id = "time_frame";
    for (int i=0; i<2; ++i) {
// 	    JointStates_msg.act_pos[i] = rdda->motor[i].motorIn.act_pos;
//    	JointStates_msg.act_vel[i] = rdda->motor[i].motorIn.act_vel;
//    	JointStates_msg.cmd_tau[i] = rdda->motor[i].motorIn.act_tau;
//    	JointStates_msg.ts_nsec = rdda->ts.nsec;
//    	JointStates_msg.ts_sec = rdda->ts.sec;
        JointStates_msg.position[i] = rdda->motor[i].motorIn.act_pos;
        JointStates_msg.velocity[i] = rdda->motor[i].motorIn.act_vel;
        ControlStates_msg.applied_effort[i] = rdda->motor[i].motorIn.act_tau;
    }
//    JointStates_msg.ext_tau[0] = rdda->psensor.analogIn.val1;
//    JointStates_msg.ext_tau[1] = rdda->psensor.analogIn.val2;
    JointStates_msg.effort[0] = rdda->psensor.analogIn.val1;
    JointStates_msg.effort[1] = rdda->psensor.analogIn.val2;

    mutex_unlock(&rdda->mutex);

//    ROS_INFO("Publish joint states act_pos[0]: %lf", JointStates_msg.act_pos[0]);
    ROS_INFO("ROS interface running...");
    rdda_joint_pub.publish(JointStates_msg);
    rdda_ctrl_pub.publish(ControlStates_msg);

}

/* Subscriber callback */
/* Comment out callback for remote test */
void RDDNode::subJointCommands_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg) {

    mutex_lock(&rdda->mutex);

    for (int i=0; i<2; ++i) {
        rdda->motor[i].rosOut.pos_ref = msg->positions[i];
    }

    ROS_INFO("Set position reference: [%lf, %lf]", msg->positions[0], msg->positions[1]);
    mutex_unlock(&rdda->mutex);
}
//void RDDNode::subJointCommands_callback(const rdda_interface::JointCommands::ConstPtr& msg) {
//
//    mutex_lock(&rdda->mutex);
//
//    for (int i=0; i<2; ++i) {
//        rdda->motor[i].rosOut.pos_ref = msg->pos_ref[i];
//	    rdda->motor[i].rosOut.vel_sat = msg->vel_sat[i];
//	    rdda->motor[i].rosOut.tau_sat = msg->tau_sat[i];
//	    rdda->motor[i].rosOut.stiffness = msg->stiffness[i];
//	    rdda->freq_anti_alias = msg->freq_anti_alias;
//    }
//
//
    //ROS_INFO("set stiffness[0]: %lf", msg->stiffness[0]);
//    mutex_unlock(&rdda->mutex);
//}

/* Service functions */
bool RDDNode::setMaxVel(rdda_interface::SetMaxVelocity::Request &req, rdda_interface::SetMaxVelocity::Response &res) {

    mutex_lock(&rdda->mutex);

    req.max_vel.resize(2);

    for (int i=0; i<2; ++i) {
        rdda->motor[i].rosOut.vel_sat = req.max_vel[i];
    }

    res.err = 0;
    ROS_INFO("Request: vel_sat = [%lf, %lf]", (double)req.max_vel[0], (double)req.max_vel[1]);
    ROS_INFO("Error indicator: %d", res.err);

    mutex_unlock(&rdda->mutex);
    return true;
}

bool RDDNode::setMaxEffort(rdda_interface::SetMaxEffort::Request &req, rdda_interface::SetMaxEffort::Response &res) {

    mutex_lock(&rdda->mutex);

    req.max_effort.resize(2);

    for (int i=0; i<2; ++i) {
        rdda->motor[i].rosOut.tau_sat = req.max_effort[i];
    }

    res.err = 0;
    ROS_INFO("Request: tau_sat = [%lf, %lf]", (double)req.max_effort[0], (double)req.max_effort[1]);

    mutex_unlock(&rdda->mutex);
    return true;
}

bool RDDNode::setStiffness(rdda_interface::SetStiffness::Request &req, rdda_interface::SetStiffness::Response &res) {

    mutex_lock(&rdda->mutex);

    req.stiffness.resize(2);

    for (int i=0; i<2; ++i) {
        rdda->motor[i].rosOut.stiffness = req.stiffness[i];
    }

    res.err = 0;
    ROS_INFO("Request: stiffness = [%lf, %lf]", (double)req.stiffness[0], (double)req.stiffness[1]);

    mutex_unlock(&rdda->mutex);
    return true;
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
        printf("shm_open error, errno(%d): %s\n", errno, strerror(errno));
        exit(1);
    }

    /* Initialise ROS node */
    ros::init(argc, argv, "rdd_node");
    printf("Launch ros interface\n");

    ros::NodeHandle node("~");
    RDDNode rdd(node, rdda);
    rdd.run();
}
