#!/usr/bin/env python

"""
This ROS node is responsible for loading a path from a file.

"""

import roslib; roslib.load_manifest('topic_logger')
import rospy

import actionlib
import topic_logger.msg


class LogTopics(object):
    """
    This is a ROS node that is responsible sending a message
    to the topicLogger Server to start logging.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('log_topics')

        # ROS params
        if (rospy.has_param('~topic_list')):
            self.topic_list = rospy.get_param('~topic_list','')
        else:
            rospy.logerr("Parameter 'topic_list' not set.")
            return;

        self.topic = rospy.get_param('~topic','/topicLogger/goal')


        topics = [x.strip() for x in self.topic_list.split(',')]
        rospy.loginfo("Topics to record: " + str(topics))
        rospy.loginfo("Creating actionlib client to send topic list to: " + self.topic)
        client=actionlib.SimpleActionClient(self.topic,topic_logger.msg.TopicLoggerAction)
        rospy.loginfo("Waiting for actionlib server.")
        client.wait_for_server()
        rospy.loginfo("Actionlib server found.")

        goal = topic_logger.msg.TopicLoggerGoal()

        goal.command = "start"
        goal.selectedTopics = topics

        client.send_goal(goal, self._handle_logging_complete, self._handle_active, self._handle_feedback)
        rospy.loginfo("Goal sent containing " + str(len(goal.selectedTopics))+ " topics.")

        rospy.spin()

    def _handle_stop_logging(self):
        self._client.cancel_goal();
        rospy.loginfo("Stop Logging")

    def _handle_feedback(self, feedback):
        pass
        #update labels with feedback - need to use signals and slots
        #print('feedback received')
        #print(type(feedback))
        #
        #self.update_status.emit(feedback)

    def _handle_logging_complete(self, goal_status, goal_result):
        rospy.loginfo(goal_status)
        rospy.loginfo(goal_result)
        rospy.loginfo('Done Logging')

    def _handle_active(self):
        rospy.loginfo('transitioned to active')


if __name__ == '__main__':
    node = LogTopics()
