#include "ros/ros.h"
#include "basic_program_topic/Message1.h"

void msgCallback(const basic_program_topic::Message1::ConstPtr& msg)
{
	ROS_INFO("recieve msg: %d", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_subscriber");

	ros::NodeHandle nh;

	ros::Subscriber basic_program_sub = nh.subscribe("basic_program_msg", 10, msgCallback);

	ros::spin();

	return 0;
}

