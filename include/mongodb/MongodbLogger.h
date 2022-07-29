#pragma once

#include <actionlib/client/simple_action_client.h>
#include <mongodb/MongoDBLoggerAction.h>

typedef actionlib::SimpleActionClient<mongodb::MongoDBLoggerAction> MongodbLoggerClient;

class MongodbLogger {
public:
    explicit MongodbLogger();

    void set_topics(const std::vector<std::string>& topics);

    /**
     * Blocking call until all topic subscribers are ready
     */
    void start();

    void end();

private:
    MongodbLoggerClient client;
    mongodb::MongoDBLoggerGoal goal;

    bool has_received_feedback = false;
    void feedback_callback(const mongodb::MongoDBLoggerFeedbackConstPtr & feedback);
};