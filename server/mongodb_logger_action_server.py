#! /usr/bin/env python
###########################################################################
#  mongodb_logger_action.py - Python based ROS Action to MongoDB logger
#
#  Created: Mon Aug 24 11:00:00 2020
#  Based on: https://github.com/strands-project/mongodb_store/blob/melodic-devel/mongodb_log/scripts/mongodb_log.py
#
#       Copyright  2020       University of Massachusetts Lowell
#                  2010-2012  Tim Niemueller [www.niemueller.de]
#                  2010-2011  Carnegie Mellon University
#                  2010       Intel Labs Pittsburgh
###########################################################################


import actionlib
from mongodb.msg import MongoDBLoggerFeedback, MongoDBLoggerResult, MongoDBLoggerAction
import rospy
import mongodb_store.util
from datetime import datetime, timedelta
import rostopic
import time
from random import randint
from multiprocessing import Process, Lock, Condition, Queue, Value, current_process, Event

QUEUE_MAXSIZE = 100
PACKAGE_NAME='mongodb_log'

MongoClient = mongodb_store.util.import_MongoClient()

class SingleTopicWorker(object):
    def __init__(self, topic, collection_name, mongodb_host, mongodb_port, msg_class, real_topic):
        self.topic = topic
        self.mongoconn = MongoClient(mongodb_host, mongodb_port)
        self.mongodb = self.mongoconn['roslog']
        self.collection = self.mongodb[collection_name]
        self.name = str(self.topic) + "_logger"
        self.subscriber = None
        self.topic_received = False
        self.msgs = []
        while not self.subscriber:
            try:
                self.subscriber = rospy.Subscriber(real_topic, msg_class, self.callback)
            except rostopic.ROSTopicIOException:
                print("FAILED to subscribe, will keep trying %s" % self.name)
                time.sleep(randint(1,10))

    def msgs_recieved(self):
        return str(self.msgs)

    def callback(self, msg):
        print("Rv from " + self.topic)
        if isinstance(msg, rospy.Message):
            self.topic_received = True
            self.msgs.append(msg)
            try:
                meta = {}
                meta["topic"]  = self.topic

                meta['latch'] = False
                meta['inserted_at'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())

                mongodb_store.util.store_message(self.collection, msg, meta)
            except InvalidDocument, e:
                print("InvalidDocument " + current_process().name + "@" + topic +": \n")
                print e
            except InvalidStringData, e:
                print("InvalidStringData " + current_process().name + "@" + topic +": \n")
                print e

    def shutdown(self):
        self.subscriber.unregister()
        self.subscriber = None
        if self.topic_received:
            return self.topic

class MongoWriter(object):
    def __init__(self, topics):
        self.topics = (str(topics)).split(':')
        self.mongodb_port = rospy.get_param("mongodb_port", 27017)
        self.mongodb_host = rospy.get_param("mongodb_host", "localhost")
        self.workers = []
        self.missing_topics = []
        self.collection_names = set()
        self.node_path = None

    def start_worker_process(self):
        for topic in self.topics:
            collection_name = mongodb_store.util.topic_name_to_collection_name(topic)
            if collection_name in self.collection_names:
                print("Two converted topic names clash: %s, ignoring topic %s"
                      % (collection_name, topic))
                continue
            try:
                print("Adding topic %s" % topic)
                w = self.create_worker(topic, collection_name)
                self.workers.append(w)
                self.collection_names |= set([collection_name])
            except Exception, e:
                print('Failed to subscribe to %s due to %s' % (topic, e))
                self.missing_topics.append(topic)

        return self.missing_topics

    def return_msgs_recieved(self):
        msgs_recieved = ""
        for worker in self.workers:
            msgs_recieved = msgs_recieved + worker.msgs_recieved()
        return msgs_recieved

    def create_worker(self, topic, collection_name):
        try:
            msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=False)
        except Exception, e:
            print('Topic %s not announced, cannot get type: %s' % (topic, e))
            raise

        if real_topic is None:
            raise rostopic.ROSTopicException('topic type was empty, probably not announced')

        w = SingleTopicWorker(topic, collection_name, self.mongodb_host, self.mongodb_port, msg_class, real_topic)

        return w

    def shutdown(self):
        topics_logged = []
        for w in self.workers:
            topics_logged.append(w.shutdown())
        return topics_logged

class MongoDBLoggerServer(object):

    feedback = MongoDBLoggerFeedback()
    result   = MongoDBLoggerResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('mongodb_logger', MongoDBLoggerAction, self.execute, False)
        self.server.start()
        rospy.loginfo("mongodb_logger action server started...")

    def execute(self, goal):
        mongo_writer = MongoWriter(goal.topics.data)
        mongo_writer.start_worker_process()

        # added to know when all ropics are subscribed
        self.feedback.msgs_received.data = "Done trying to subscribe all topics"
        self.server.publish_feedback(self.feedback)

        self.result.topics_logged = ""
        rate = rospy.Rate(1)
        while not self.server.is_preempt_requested():
            self.feedback.msgs_received.data = mongo_writer.return_msgs_recieved()
            self.server.publish_feedback(self.feedback)
            rate.sleep()
        rospy.loginfo('MongoDBLogger has been cancelled')
        self.result.topics_logged = mongo_writer.shutdown()
        self.server.set_preempted(self.result)

if __name__ == '__main__':
    rospy.init_node('mongodb_logger_server')
    server = MongoDBLoggerServer()
    rospy.spin()
