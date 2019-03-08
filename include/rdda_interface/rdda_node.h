#ifndef RDDA_NODE_H
#define RDDA_NODE_H

/* C++ headers */
#include <pthread.h>

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "rdda_interface/JointCommands.h"
#include "rdda_interface/JointStates.h"

/* C headers */
extern "C" {
#include "./shared_data.h" 
#include "./shared_memory.h"
};

class RDDNode {
 public:
    explicit RDDNode(ros::NodeHandle& node, shared_in_t *in, shared_out_t *out);

    ~RDDNode();

    void run();

 private:
    ros::NodeHandle nh_;
    ros::Subscriber rdda_joint_sub;
    ros::Publisher rdda_joint_pub;

    shared_in_t *shared_in;
    shared_out_t *shared_out;

    void pubJointStates();
    void subJointCommands_callback(const rdda_interface::JointCommands::ConstPtr& msg);
};

#endif /* RDDA_NODE */
