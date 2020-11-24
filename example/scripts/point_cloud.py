#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan, PointCloud, Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class point_pub:
    def __init__(self):
        rospy.init_node('point_pub', anonymous=True)

        self.point_pub = rospy.Publisher('/point', PointCloud, queue_size=1)
        self.point_msg = PointCloud()
        self.point_msg.header.frame_id = '/map'

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('example')

        for i in range(1, 352):
            if i < 10:
                num = '00' + str(i)
            elif 9 < i < 100:
                num = '0' + str(i)
            elif 99 < i < 1000:
                num = str(i)

            full_path = pkg_path + '/path' + '/kcity_PM0138' + \
                '/A1LANE_CenterLine_0' + num + '.csv'

            self.f = open(full_path, 'r')

            lines = self.f.readlines()[8:]

            for line in lines:
                tmp = line.split()
                read_pose = Point32()
                read_pose.x = float(tmp[0])-302459.942
                read_pose.y = float(tmp[1])-4122635.537
                read_pose.z = float(tmp[2])-28.989999771118164

                self.point_msg.points.append(read_pose)

        for i in range(1, 187):
            if i < 10:
                num = '00' + str(i)
            elif 9 < i < 100:
                num = '0' + str(i)
            elif 99 < i < 1000:
                num = str(i)

            full_path = pkg_path + '/path' + '/kcity_PM0138' + \
                '/A1LANE_NormalLane_0' + num + '.csv'

            self.f = open(full_path, 'r')

            lines = self.f.readlines()[8:]

            for line in lines:
                tmp = line.split()
                read_pose = Point32()
                read_pose.x = float(tmp[0])-302459.942
                read_pose.y = float(tmp[1])-4122635.537
                read_pose.z = float(tmp[2])-28.989999771118164

                self.point_msg.points.append(read_pose)

        for i in range(1, 561):
            if i < 10:
                num = '00' + str(i)
            elif 9 < i < 100:
                num = '0' + str(i)
            elif 99 < i < 1000:
                num = str(i)

            full_path = pkg_path + '/path' + '/kcity_PM0138' + \
                '/A1LANE_RoadEdge_0' + num + '.csv'

            self.f = open(full_path, 'r')

            lines = self.f.readlines()[8:]

            for line in lines:
                tmp = line.split()
                read_pose = Point32()
                read_pose.x = float(tmp[0])-302459.942
                read_pose.y = float(tmp[1])-4122635.537
                read_pose.z = float(tmp[2])-28.989999771118164

                self.point_msg.points.append(read_pose)

        self.f.close()

        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            self.point_pub.publish(self.point_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        test_track = point_pub()
    except rospy.ROSInterruptException:
        pass
