#include <ros/ros.h>
#include "caddy_manipulation/CaddyPicking.h"
#include <actionlib/client/simple_action_client.h>
#include <mongodb/MongoDBLoggerAction.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mongodb_logger_client");

    actionlib::SimpleActionClient<mongodb::MongoDBLoggerAction> client("mongodb_logger");
    client.waitForServer();

    mongodb::MongoDBLoggerGoal goal;
    goal.topics.data = "/gripper_controller/gripper_action/goal:/arm_with_torso_controller/follow_joint_trajectory/goal";

    client.sendGoal(goal);

    CaddyPicking();

    client.cancelGoal();

    return 0;
}

