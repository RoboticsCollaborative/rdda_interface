#include "rdda_interface/rdda_interface.h"

using namespace std;

/* RDDNode constructor */
RDDNode::RDDNode(ros::NodeHandle &node, Rdda *rddaptr) {
    nh_ = node;
    rdda = rddaptr;

    rdda_joint_sub = nh_.subscribe("joint_cmds", 1, &RDDNode::subJointCommands_callback, this);

    rdda_joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    rdda_ctrl_pub = nh_.advertise<rdda_interface::ControlState>("ctrl_states", 1);

    rdda_maxvel_srv = nh_.advertiseService("set_max_vel", &RDDNode::setMaxVel, this);
    rdda_maxeff_srv = nh_.advertiseService("set_max_eff", &RDDNode::setMaxEffort, this);
    rdda_stiff_srv = nh_.advertiseService("set_stiff", &RDDNode::setStiffness, this);

    rdda_homing_cli = nh_.serviceClient<rdda_interface::Homing>("/rdda_interface/homing");
}

RDDNode::~RDDNode() = default;

/* Initialize interface with ROS parameters. */
void RDDNode::initConfigParams() {
    double freq, stiff[2], max_vel[2], max_eff[2];
    bool need_homing;

    mutex_lock(&rdda->mutex);

    for (int i=0; i<2; ++i) {
        if (ros::param::get("~stiff", stiff[i])) {
            rdda->motor[i].rosOut.stiffness = stiff[i];
        }
        if (ros::param::get("~max_vel", max_vel[i])) {
            rdda->motor[i].rosOut.vel_sat = max_vel[i];
        }
        if (ros::param::get("~max_eff", max_eff[i])) {
            rdda->motor[i].rosOut.tau_sat = max_eff[i];
        }
    }
    if (ros::param::get("~anti_alias_freq", freq)) {
        rdda->freq_anti_alias = freq;
    }

    mutex_unlock(&rdda->mutex);

    if (ros::param::get("~need_homing", need_homing)) {
        std::vector<double> origins;
        rdda_interface::Homing homing_srv;
        homing_srv.request.need_homing = need_homing;
        if (rdda_homing_cli.call(homing_srv)) {
            for (int i=0; i<2; ++i) {
                origins[i] = homing_srv.response.origins[i];
                ros::param::set("~origins", origins);
            }
            ROS_INFO("Joint Origins: [%lf, %lf]", origins[0], origins[1]);
        }
    }
}

/* Publish joint state */
void RDDNode::pubJointStates() {

    sensor_msgs::JointState JointStates_msg;
    JointStates_msg.position.resize(2);
    JointStates_msg.velocity.resize(2);
    JointStates_msg.effort.resize(2);
    rdda_interface::ControlState ControlStates_msg;
    ControlStates_msg.applied_effort.resize(2);

    mutex_lock(&rdda->mutex);

////    JointStates_msg.header.frame_id = "time_frame";
    for (int i=0; i<2; ++i) {
        JointStates_msg.header.stamp.sec = rdda->ts.sec;
        JointStates_msg.header.stamp.nsec = rdda->ts.nsec;
        JointStates_msg.position[i] = rdda->motor[i].motorIn.act_pos;
        JointStates_msg.velocity[i] = rdda->motor[i].motorIn.act_vel;
        ControlStates_msg.applied_effort[i] = rdda->motor[i].motorIn.act_tau;
    }
    JointStates_msg.effort[0] = rdda->psensor.analogIn.val1;
    JointStates_msg.effort[1] = rdda->psensor.analogIn.val2;

    mutex_unlock(&rdda->mutex);

    rdda_joint_pub.publish(JointStates_msg);
    rdda_ctrl_pub.publish(ControlStates_msg);

//    ROS_INFO("Publish joint states act_pos[0]: %lf\r", JointStates_msg.position[0]);
//    ROS_INFO("Publish freq[0]: %lf\r", rdda->freq_anti_alias);
    ROS_INFO("ROS interface running...");
}

/* Subscriber callback */
/* Comment out callback for remote test */
void RDDNode::subJointCommands_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &JointCommands_msg) {

    mutex_lock(&rdda->mutex);

    for (int i=0; i<2; ++i) {
        rdda->motor[i].rosOut.pos_ref = JointCommands_msg->positions[i];
    }

    ROS_INFO("Set position reference: [%lf, %lf]", JointCommands_msg->positions[0], JointCommands_msg->positions[1]);
    mutex_unlock(&rdda->mutex);
}

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
    ros::Rate loop_rate(20);
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
    if (rdda == nullptr) {
        fprintf(stderr, "Init rdda failed.\n");
        printf("shm_open error, errno(%d): %s\n", errno, strerror(errno));
        exit(1);
    }

    /* Initialise ROS node */
    ros::init(argc, argv, "rdda_interface");
    printf("Launch ros interface\n");

    ros::NodeHandle node("~");
    RDDNode rdd(node, rdda);
    rdd.initConfigParams();
    rdd.run();
}
