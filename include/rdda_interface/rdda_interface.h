#ifndef RDDA_NODE_H
#define RDDA_NODE_H

/* C++ headers */
#include <pthread.h>
#include <string.h>
#include <errno.h>

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
//#include "rdda_interface/JointCommands.h"
//#include "rdda_interface/JointStates.h"
#include "rdda_interface/ControlState.h"

#include "rdda_interface/SetMaxVelocity.h"
#include "rdda_interface/SetMaxEffort.h"
#include "rdda_interface/SetStiffness.h"


/* C headers */
extern "C" {
#include "shm_data.h"
#include "shm.h"
};

class RDDNode {
 public:
    explicit RDDNode(ros::NodeHandle& node, Rdda *rdda);

    ~RDDNode();

    void run();
    void initConfigParams();

 private:
    ros::NodeHandle nh_;
    ros::Subscriber rdda_joint_sub;
    ros::Publisher rdda_joint_pub;
    ros::Publisher rdda_ctrl_pub;

    ros::ServiceServer rdda_maxvel_srv;
    ros::ServiceServer rdda_maxeff_srv;
    ros::ServiceServer rdda_stiff_srv;

    Rdda *rdda;

    void pubJointStates();
//    void subJointCommands_callback(const rdda_interface::JointCommands::ConstPtr &msg);
    void subJointCommands_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);

    bool setMaxVel(rdda_interface::SetMaxVelocity::Request &req, rdda_interface::SetMaxVelocity::Response &res);
    bool setMaxEffort(rdda_interface::SetMaxEffort::Request &req, rdda_interface::SetMaxEffort::Response &res);
    bool setStiffness(rdda_interface::SetStiffness::Request &req, rdda_interface::SetStiffness::Response &res);
};

#endif /* RDDA_NODE */
