#include <boost/algorithm/string/join.hpp>

#include "mongodb/MongodbLogger.h"
#include "mongodb/MongoDBLoggerFeedback.h"

MongodbLogger::MongodbLogger() : client("mongodb_logger"){
    ROS_INFO("[MongodbLogger] Waiting for server...");
    client.waitForServer();
}

void MongodbLogger::set_topics(const std::vector<std::string>& topics) {
    std::string topics_string = boost::algorithm::join(topics, ":");
    ROS_INFO_STREAM("[MongodbLogger] Setting topics to " << topics_string);
    goal.topics.data = topics_string;
}

void MongodbLogger::start() {
    has_received_feedback = false;
    client.sendGoal(goal,
                    MongodbLoggerClient::SimpleDoneCallback(),
                    MongodbLoggerClient::SimpleActiveCallback(),
                    boost::bind(&MongodbLogger::feedback_callback, this, _1));

    // wait until all subscribers are ready, so no messages sent right after start() are lost
    ROS_INFO_STREAM("Waiting until mongodb logger subscribers are ready...");
    while ( ! has_received_feedback) {
        ros::Duration(0.05).sleep();
    }
    ROS_INFO_STREAM("Ready.");
}

void MongodbLogger::feedback_callback(const mongodb::MongoDBLoggerFeedbackConstPtr & feedback) {
    has_received_feedback = true;
}

void MongodbLogger::end() {
    has_received_feedback = false;
    client.cancelGoal();
}
