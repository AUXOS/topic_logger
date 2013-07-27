#!/usr/bin/env python

"""
This ROS node is responsible for loading a path from a file.

"""

import roslib; roslib.load_manifest('topic_logger')
import rospy
import time

import actionlib
import auxos_messages.msg


class LogTopics(object):
    """
    This is a ROS node that is responsible sending a message
    to the topicLogger Server to start logging.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('log_topics')
        rospy.on_shutdown(self.on_shutdown)

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
        self._client=actionlib.SimpleActionClient(self.topic,auxos_messages.msg.TopicLoggerAction)
        rospy.loginfo("Waiting for actionlib server.")
        self._client.wait_for_server()
        rospy.loginfo("Actionlib server found.")

        goal = auxos_messages.msg.TopicLoggerGoal()

        goal.command = "start"
        goal.selectedTopics = topics

        self._client.send_goal(goal, self._handle_logging_complete, self._handle_active, self._handle_feedback)
        rospy.loginfo("Goal sent containing " + str(len(goal.selectedTopics))+ " topics.")

        time.sleep(10)
        self.on_shutdown()

        #rospy.spin()

    def on_shutdown(self):
        # send stop
        rospy.loginfo("Stopping.")
        self._client.cancel_all_goals()
        rospy.loginfo("Cancelled goals.")

    def _handle_feedback(self, feedback):
        #update labels with feedback - need to use signals and slots
        print('feedback received')
        print(feedback)
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
