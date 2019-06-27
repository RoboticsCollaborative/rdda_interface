#ifndef RDDA_NODE_H
#define RDDA_NODE_H

/* C++ headers */
#include <pthread.h>
#include <string.h>
#include <errno.h>

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "rdda_interface/JointCommands.h"
#include "rdda_interface/JointStates.h"

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

 private:
    ros::NodeHandle nh_;
    ros::Subscriber rdda_joint_sub;
    ros::Publisher rdda_joint_pub;

    Rdda *rdda;

    void pubJointStates();
    void subJointCommands_callback(const rdda_interface::JointCommands::ConstPtr& msg);
};

#endif /* RDDA_NODE */
