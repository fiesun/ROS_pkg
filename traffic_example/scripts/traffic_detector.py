#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import rospkg

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class TRAFFICDetector:
    def __init__(self):

        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.callback)

        self.traffic_msg = String()

        self.signal_pub = rospy.Publisher(
            "/traffic_light", String, queue_size=10)

        self.img_hsv = None

    def callback(self, msg):
        try:
            np.arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np.arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    def detect_signal(self):

        h = self.img_hsv.shape[0]
        w = self.img_hsv.shape[1]

        lower_sig_y = np.array([20, 230, 230])
        upper_sig_y = np.array([30, 255, 255])

        lower_sig_r = np.array([0, 230, 230])
        upper_sig_r = np.array([10, 255, 255])

        lower_sig_g = np.array([50, 230, 230])
        upper_sig_g = np.array([60, 255, 255])

        img_r = cv2.resize(cv2.inRange(
            self.img_hsv, lower_sig_r, upper_sig_r), (w / 2, h / 2))

        img_y = cv2.resize(cv2.inRange(
            self.img_hsv, lower_sig_y, upper_sig_y), (w / 2, h / 2))

        img_g = cv2.resize(cv2.inRange(
            self.img_hsv, lower_sig_g, upper_sig_g), (w / 2, h / 2))

        img_r[int(h / 3 / 2):, :] = 0
        img_y[int(h / 3 / 2):, :] = 0
        img_g[int(h / 3 / 2):, :] = 0

        pix_r = cv2.countNonZero(img_r)
        pix_y = cv2.countNonZero(img_y)
        pix_g = cv2.countNonZero(img_g)

        pix_max = np.max([pix_r, pix_y, pix_g])
        idx_s = np.argmax([pix_r, pix_y, pix_g])

        if pix_max > 40:

            if idx_s == 0:
                self.traffic_msg.data = "RED"

            elif idx_s == 1:
                self.traffic_msg.data = "YELLOW"

            else:
                self.traffic_msg.data = "GREEN"

        else:

            self.traffic_msg.data = "None"

    def pub_signal(self):

        self.signal_pub.publish(self.traffic_msg)


if __name__ == '__main__':

    rospy.init_node("traffic_detector", anonymous=True)

    traffic_detector = TRAFFICDetector()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if traffic_detector.img_hsv is not None:

            traffic_detector.detect_signal()

            traffic_detector.pub_signal()

            rate.sleep()
