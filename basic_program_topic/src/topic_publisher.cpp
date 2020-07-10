#include "ros/ros.h"
#include "basic_program_topic/Message1.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_publisher");
	ros::NodeHandle nh;

	ros::Publisher basic_program_pub = nh.advertise<basic_program_topic::Message1>("basic_program_msg", 100);

	ros::Rate loop_rate(10);

	basic_program_topic::Message1 msg;
	int count = 0;
	while (ros::ok())
	{
		msg.data = count;

		ROS_INFO("send msg = %d", msg.data);

		basic_program_pub.publish(msg);

		loop_rate.sleep();

		++count;
	}
	return 0;
}
