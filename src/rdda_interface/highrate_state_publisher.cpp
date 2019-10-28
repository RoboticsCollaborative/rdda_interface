#include "rdda_interface/highrate_state_publisher.h"

using namespace std;

/* HrsPublisher constructor */
HrsPublisher::HrsPublisher(ros::NodeHandle &node, Rdda *rddaptr) {
    nh_ = node;
    rdda = rddaptr;

    highrate_joint_pub = nh_.advertise<sensor_msgs::JointState>("highrate_joint_states", 1);
    highrate_ctrl_pub = nh_.advertise<rdda_interface::ControlState>("highrate_control_states", 1);
}

HrsPublisher::~HrsPublisher() = default;

/* Publish highrate joint state */
void HrsPublisher::pubHighrateJointStates() {

    sensor_msgs::JointState JointStates_msg;
    JointStates_msg.position.resize(2);
    JointStates_msg.velocity.resize(2);
    JointStates_msg.effort.resize(2);
    rdda_interface::ControlState ControlStates_msg;
    ControlStates_msg.applied_effort.resize(2);

    mutex_lock(&rdda->mutex);

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

    highrate_joint_pub.publish(JointStates_msg);
    highrate_ctrl_pub.publish(ControlStates_msg);
}

/* Run loop */
void HrsPublisher::run() {
    ros::Rate loop_rate(500);
    while (ros::ok()) {
        /* Publisher (wrap) */
        pubHighrateJointStates();
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
    ros::init(argc, argv, "highrate_state_publisher");
    printf("Launch Highrate State Publisher\n");

    ros::NodeHandle node("~");
    HrsPublisher hrs(node, rdda);
    hrs.run();
}