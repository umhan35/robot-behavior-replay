#include <ros/ros.h>
#include "mongodb/ClockReMapper.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "undo_gazebo_clock_remap");

    ClockReMapper clockReMapper;

    return 0;
}