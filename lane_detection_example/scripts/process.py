#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Float32


class Process:
    def __init__(self):
        rospy.init_node('process', anonymous=True)

        rospy.Subscriber("/chatter", Float32, self.chatter_callback)

        rospy.Subscriber("/pk/commands/motor/speed",
                         Float64, self.pk_speed_callback)
        rospy.Subscriber("/pk/commands/servo/position",
                         Float64, self.pk_position_callback)

        rospy.Subscriber("/ln/commands/motor/speed",
                         Float64, self.ln_speed_callback)
        rospy.Subscriber("/ln/commands/servo/position",
                         Float64, self.ln_position_callback)

        rospy.Subscriber("/park/state", Float64, self.park_callback)

        self.motor_pub = rospy.Publisher(
            '/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher(
            '/commands/servo/position', Float64, queue_size=1)

        self.is_sensor = False

        self.ln_speed_value = 0
        self.ln_position_value = 0

        self.pk_speed_value = 0
        self.pk_position_value = 0

        self.park_state = 0

        motor_msg = Float64()
        servo_msg = Float64()

        rate = rospy.Rate(30)

	self.st = 0

        while not rospy.is_shutdown():
            if self.is_sensor == True:
                self.motor_pub.publish(motor_msg)
                self.servo_pub.publish(servo_msg)

                if self.st == 0 or self.park_state == 1:
                    motor_msg = self.ln_speed_value
                    servo_msg = self.ln_position_value
                elif self.st == 1:
                    motor_msg = self.pk_speed_value
                    servo_msg = self.pk_position_value
            rate.sleep()

    def park_callback(self, msg):
        self.park_state = msg.data

    def chatter_callback(self, msg):
        self.is_sensor = True
        self.sensor_value = msg.data
	if self.sensor_value <= 1000:
	    self.st = 1

    def pk_speed_callback(self, msg):
        self.pk_speed_value = msg.data

    def pk_position_callback(self, msg):
        self.pk_position_value = msg.data

    def ln_speed_callback(self, msg):
        self.ln_speed_value = msg.data

    def ln_position_callback(self, msg):
        self.ln_position_value = msg.data


if __name__ == '__main__':
    try:
        test_track = Process()
    except rospy.ROSInterruptException:
        pass
