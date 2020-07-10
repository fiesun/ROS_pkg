#!/usr/bin/env python

import rospy
from topic_tutorial_2.msg import MyMsgs

NAME_TOPIC = '/msgs_talk'
NAME_NODE = 'pub_node1'

if __name__ == '__main__':
    
    rospy.init_node(NAME_NODE, anonymous = True)

    pub = rospy.Publisher(NAME_TOPIC, MyMsgs, queue_size = 10)

    rate = rospy.Rate(10) # 10hz

    msgs_pub = MyMsgs()

    while not rospy.is_shutdown():

        msgs_pub.x = [1, 2, 3, 4]
        msgs_pub.y = [2, 3, 4, 5]

        pub.publish(msgs_pub)

        rate.sleep()
