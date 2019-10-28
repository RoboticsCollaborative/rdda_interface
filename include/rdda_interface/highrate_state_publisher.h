#ifndef HIGHRATE_STATE_PUBLISHER_H
#define HIGHRATE_STATE_PUBLISHER_H

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include "rdda_interface/ControlState.h"

/* C headers */
extern "C" {
#include "shm_data.h"
#include "shm.h"
};

class HrsPublisher {
public:
    explicit HrsPublisher(ros::NodeHandle& node, Rdda *rdda);

    ~HrsPublisher();

    void run();

private:
    ros::NodeHandle nh_;

    ros::Publisher highrate_joint_pub;
    ros::Publisher highrate_ctrl_pub;

    Rdda *rdda;

    void pubHighrateJointStates();
};

#endif //RDDA_INTERFACE_HIGHRATE_STATE_PUBLISHER_H
