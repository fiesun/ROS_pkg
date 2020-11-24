#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos, sin, pi
from geometry_msgs.msg import Point32

class parking_guardrail:
    def __init__(self):
        rospy.init_node('parking_guardrail', anonymous = True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.motor_pub = rospy.Publisher('commands/motor/speed', Float64, queue_size = 1)
        self.servo_pub = rospy.Publisher('commands/servo/position', Float64, queue_size = 1) 
        self.pcd_pub = rospy.Publisher('laser2pcd', PointCloud, queue_size = 1)

        self.init_second = rospy.get_time()
    
        rate = rospy.Rate(30)

        self.s = 1

        while not rospy.is_shutdown():
            
            rate.sleep()

    def laser_callback(self, msg):
        pcd = PointCloud()
        motor_msg = Float64()
        servo_msg = Float64()
        pcd.header.frame_id = msg.header.frame_id

        angle = 0

        for r in msg.ranges:
            tmp_point = Point32()
            tmp_point.x = r * cos(angle)
            tmp_point.y = r * sin(angle)
            
            angle = angle + (1.0 / 180 * pi)

            if r < 12:
                pcd.points.append(tmp_point)

        count_l = 0
        count_r = 0
        count_x = 0
        count_y = 0

        for point in pcd.points:
            if point.x > 0 and point.x < 3 and point.y > 0 and point.y < 0.7:
                count_l += 1
            elif point.x > 0 and point.x < 3 and point.y > -0.7 and point.y < 0:
                count_r += 1
            elif point.x > 0 and point.x < 3:
                count_x += 1
            elif point.y < 0 and point.y > -3:
                count_y += 1

        self.second = rospy.get_time()

        if self.s == 1:
            servo_msg.data = 0.85
            motor_msg.data = 1000
            if count_y <= 52:
                self.s = 2
        elif self.s == 2:
            servo_msg.data = 0.15
            motor_msg.data = 1000
            if count_y >= 92:
                self.s = 3
        elif self.s == 3:
            servo_msg.data = 0.5304
            motor_msg.data = 0

        print(count_l, count_r, count_x, count_y, self.s)

        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)
        self.pcd_pub.publish(pcd)

if __name__ == '__main__':
    try:
        test_track = parking_guardrail()
    except rospy.ROSInterruptException:
        pass