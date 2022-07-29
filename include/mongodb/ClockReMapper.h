#pragma once

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

class ClockReMapper {
public:
    ros::NodeHandle nh_;

    ros::Subscriber sub;

    ros::Publisher pub;

    rosgraph_msgs::Clock clock;

    ClockReMapper();

    void callback(const rosgraph_msgs::Clock msg);

};

