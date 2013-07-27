/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Robert Bosch LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

//\Author Ralf Kempf, Robert Bosch LLC revised Sarah Osentoski for electric

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <auxos_messages/TopicLoggerAction.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <vector>
#include <topic_logger/recorder.h>
#include <unistd.h>

using namespace std;

class TopicLoggerAction
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<auxos_messages::TopicLoggerAction> as_;

	string action_name_;
	//create messages that are used to published feedback/result
	auxos_messages::TopicLoggerFeedback feedback_;
	auxos_messages::TopicLoggerResult result_;

	rosbag::Recorder recorder;

public:

	TopicLoggerAction(string name):
		as_(nh_, name, boost::bind(&TopicLoggerAction::executeCB, this, _1), false),
		action_name_(name)
		{
				as_.start();
				ROS_INFO("Server is up");
		}

	~TopicLoggerAction(void)
	{
	}

	void executeCB(const auxos_messages::TopicLoggerGoalConstPtr &goal)
	{

		//set rate for publishing feedback
		ros::Rate r(1);

		if (goal->selectedTopics.size()==0)
			recorder.options_.record_all = true;
		else 
			recorder.options_.topics = goal->selectedTopics;
		recorder.run();

		feedback_.filesize = 0.0;
		feedback_.target_filename = recorder.GetTargetFilename().c_str();
		while (ros::ok())
		{
			//check if preemption is requested
			if (as_.isPreemptRequested())
			{
				break;
			}
			else
			{
				//publish current filesize as feedback
				feedback_.filesize = recorder.GetFilesize();
				feedback_.target_filename = recorder.GetWriteFilename().c_str();
				as_.publishFeedback(feedback_);
				r.sleep();
			}
		}

		ROS_INFO("Closing log file ...");
		recorder.stop();
		result_.write_filename = recorder.GetTargetFilename().c_str();
		as_.setSucceeded(result_);

	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "topicLogger");
	ros::NodeHandle nh;

	//get local work directory from param server and change LWD
	std::string localWD;
	if(nh.getParam("localWorkDirectory_topicLogger", localWD))
	{
		//change the work directory so that the local directory for saving the .bag file is correct
		int i;
		i = chdir(localWD.c_str());
		if (i != 0)
			ROS_INFO("Changing the work directory failed");
		else ROS_INFO("Local work directory is: %s", localWD.c_str());
	}
	else ROS_INFO("Failed to load 'localWorkDirectory_topicLogger'.");

	TopicLoggerAction topicLogger(ros::this_node::getName());
	ros::MultiThreadedSpinner s(10);
	ros::spin();

	return 0;
}
