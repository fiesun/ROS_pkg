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

is_in_lane = 0


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

        self.speed_value = True

    def callback(self, msg):
        try:
            np.arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np.arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        ret, self.img_wlane = cv2.threshold(
            img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # cv2.imshow("Image window",  self.img_wlane)
        # cv2.waitKey(1)


def park_callback(msg):
    global is_in_lane
    is_in_lane = msg.data
    print(is_in_lane)


if __name__ == '__main__':

    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath, 'sensor/sensor_param.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node("lane_detector", anonymous=True)

    park_sub = rospy.Subscriber("/park/state", Float64, park_callback)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)

    curve_learner = CURVEFit(order=3)

    ctrller = purePursuit(lfd=0.7)

    rate = rospy.Rate(20)
    is_init = False

    while not rospy.is_shutdown():
        if is_init == False and is_in_lane == 1:
            is_init = True
            image_parser = IMGParser()
            bev_op = BEVTransform(params_cam=params_cam)
            curve_learner = CURVEFit(order=3)

        if image_parser.img_wlane is not None:

            img_warp = bev_op.warp_bev_img(image_parser.img_wlane)
            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            x_pred, y_pred_l, y_pred_r = curve_learner.fit_curve(lane_pts)

            ctrller.steering_angle(x_pred, y_pred_l, y_pred_r)
            ctrller.pub_cmd()

            curve_learner.write_path_msg(x_pred, y_pred_l, y_pred_r)

            curve_learner.pub_path_msg()

            xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

            img_warp1 = draw_lane_img(img_warp, xyl[:, 0].astype(np.int32),
                                      xyl[:, 1].astype(np.int32),
                                      xyr[:, 0].astype(np.int32),
                                      xyr[:, 1].astype(np.int32))

            cv2.imshow("Image window", img_warp1)
            cv2.waitKey(1)

            rate.sleep()
