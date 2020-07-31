#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from utils import warp_image, BEVTransform, CURVEFit, draw_lane_img, purePursuit, STOPLineEstimator


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.callback)

        self.source_prop = np.float32([[0.05, 0.65],
                                       [0.5 - 0.15, 0.52],
                                       [0.5 + 0.15, 0.52],
                                       [1 - 0.05, 0.65]
                                       ])

        self.img_wlane = None

    def callback(self, msg):
        try:
            np.arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np.arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([20, 0, 255])
        upper_wlane = np.array([35, 15, 255])
        self.img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)


if __name__ == '__main__':

    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_param.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node("stopline_detector", anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    sline_detector = STOPLineEstimator()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.img_wlane is not None:

            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            sline_detector.get_x_points(lane_pts)
            sline_detector.estimate_dist(0.3)

            # sline_detector.visualize_dist()

            sline_detector.pub_sline()

            rate.sleep()
