#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

class IMGParser:
	def __init__(self):
		# IMGParser 노드 생성
		rospy.init_node('IMGParser', anonymous=True)

		# 카메라 영상 받는 Subscriber	
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)

		# 영상 오프셋 보내는 Publisher
		self.offset_pub = rospy.Publisher('/offset', Float64, queue_size=1)
		
		# 오프셋 메시지 타입 지정
		self.offset_msg = Float64()
	
		# 이미지 우측, 좌측, 중앙 포인트 지정 변수 초기화
		self.img_wlane = None
		self.point_height = None
		self.left_weight = None
		self.right_weight = None
		self.center_weight = None

		# 이전 좌, 우 포인트 값 기억 변수
		self.pre_right = 0
		self.pre_left = 0

		# 10hz 딜레이
		rate = rospy.Rate(10)
		# 반복 실행
		while not rospy.is_shutdown():
			if self.img_wlane is not None:
				self.offset_pub.publish(self.offset_msg)
				
				# 화면상 우측, 좌측, 중앙 포인트 표시
				cv2.circle(self.roi_img, (self.left_weight,self.point_height),3,(255,0,0),10)
				cv2.circle(self.roi_img, (self.right_weight,self.point_height),3,(0,0,255),5)
				cv2.circle(self.roi_img, (self.center_weight,self.point_height),3,(0,255,0),5)

				# 카메라 영상 확인용 출력
				cv2.imshow('test',self.img_wlane)
				cv2.imshow("camera", self.roi_img)
				cv2.waitKey(1)

			# 반복 딜레이 10hz
			rate.sleep()

	# 카메라 영상 받아오는 콜백 함수
	def callback(self, msg):
		np_arr = np.fromstring(msg.data, np.uint8)
		img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		# 영상 크기 조정 (roi 설정)
		self.roi_img =img_bgr[240:480, 0:640]
		# 그레이 스케일
		img_gray = cv2.cvtColor(self.roi_img, cv2.COLOR_BGR2GRAY)
		# 이진화 
		ret, self.img_wlane = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
		# 영상 높이 넓이 확인
		height, weight = self.img_wlane.shape

		# 넓이의 중앙, 높이의 4/5 지점 포인트 위치 표시
		point_weight = weight / 2
		self.point_height = height * 4/5
		self.left_weight = point_weight
		self.right_weight = point_weight

		# 좌, 우 차선 확인용 변수
		left_check = False
		right_check = False

		while left_check == False or right_check == False :
			# 좌측 차선 확인
			if self.img_wlane[self.point_height][self.left_weight] != 0 or self.left_weight <= 0:
				left_check = True
			else:
				self.left_weight -= 1
			# 우측 차선 확인
			if self.img_wlane[self.point_height][self.right_weight] != 0 or self.right_weight >= weight-1:
				right_check= True
			else:	
				self.right_weight += 1

		# 차선 미인식 상황에서 이전 차선 값으로 지정
		if self.right_weight-self.left_weight < 200:
			self.right_weight = self.pre_right
			self.left_weight =  self.pre_left
		else:
			self.pre_right = self.right_weight 
			self.pre_left = self.left_weight

		# 화면 중앙 값과 차선 중앙 값 차이 계산
		self.center_weight = (self.right_weight  + self.left_weight) / 2

		# 오프셋 publish 데이터 지정
		self.offset_msg.data = (weight/2) - self.center_weight

# 메인 함수, IMGParser 클래스 실행
if __name__ == '__main__':
	try:
		image_parser = IMGParser()
	except rospy.ROSInterruptException:
		pass