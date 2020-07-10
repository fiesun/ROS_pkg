#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

name_node = 'turtlesim_position_contrl'

class POSITIONController:

    def __init__(self, ref = [1,1]):
        self.ref = ref
        self.dist_move = 0
        self.cmd_vel_msgs = Twist()
        self.cmd_vel_topic = '/turtle1/cmd_vel'
        self.pose_topic = '/turtle1/pose'

        self.x = None
        self.y = None
        self.yaw = None

        self.x0 = None
        self.y0 = None
        self.yaw0 = None

        self.k_lin = 1
        self.k_rot = 4

        self.sub = rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 10)

    def pose_callback(self, msgs):
        self.x = msgs.x
        self.y = msgs.y
        self.yaw = msgs.theta

    def pub_cmd_msgs(self):
        err_dis = math.sqrt((self.x - self.ref[0])**2 + (self.y - self.ref[1]) ** 2)
        yaw_d = math.atan2(self.ref[1] - self.y, self.ref[0] - self.x)
        err_yaw = yaw_d - self.yaw

        if err_dis >= 0.1:
            self.cmd_vel_msgs.linear.x = err_dis * self.k_lin
            self.cmd_vel_msgs.angular.z = err_yaw * self.k_rot

        else:
            self.cmd_vel_msgs.linear.x = 0
            self.cmd_vel_msgs.angular.z = 0

        self.pub.publish(self.cmd_vel_msgs)

    def init_pose(self):
        self.x0 = self.x
        self.y0 = self.y
        self.yaw0 = self.yaw

if __name__ == '__main__':
    rospy.init_node(name_node)

    p_ctrl = POSITIONController(ref = [3,10])

    time.sleep(2)

    rate = rospy.Rate(10) # 10hz

    p_ctrl.init_pose()

    while not rospy.is_shutdown():
        p_ctrl.pub_cmd_msgs()

        rate.sleep()