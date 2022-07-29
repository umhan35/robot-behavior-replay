#include <rosgraph_msgs/Clock.h>
#include <ros/ros.h>
#include "mongodb/ClockReMapper.h"


ClockReMapper::ClockReMapper() {
    sub = nh_.subscribe("clock_temp", 1000, &ClockReMapper::callback, this);
    pub = nh_.advertise<rosgraph_msgs::Clock>("clock", 1000);
    ros::spin();
}

void ClockReMapper::callback(const rosgraph_msgs::Clock msg) {
    clock = msg;
    pub.publish(clock);
}


