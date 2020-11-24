#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Float64

class Pure_pursuit:
	def __init__(self):
    	# Pure_pursuit 노드 생성
		rospy.init_node('Pure_pursuit', anonymous=True)

		# 영상 오프셋 받는 Subscriber
		rospy.Subscriber("/offset", Float64, self.offset_callback)

		# 모터 스피드, 스티어링 값 보내는 Publisher
		self.motor_pub = rospy.Publisher('commands/motor/speed', Float64, queue_size=1)
		self.servo_pub = rospy.Publisher('commands/servo/position', Float64, queue_size=1)
		
		# 모터 스피드, 스티어링 데이터 타입 지정
		self.motor_msg=Float64()
		self.servo_msg=Float64()

		# 오프셋 초기화
		self.is_offset = False
		self.offset = 0

		# 스티어링, 스티어링 오프셋 초기화
		self.steering = 0
		self.servo_gain = -0.015
		self.steering_angle_to_servo_offset = 0.5304

		# 20hz 딜레이
		rate = rospy.Rate(20)
		# 반복실행
		while not rospy.is_shutdown():
			if self.is_offset ==True :
				# 스티어링 값 지정
				if self.offset == 0:
					self.steering = 0
				# 스티어링 최소, 최대 값 지정 
				else:
					self.steering = self.offset / 8
					if self.steering >= 20:
						self.steering = 22
					elif self.steering <= -20:
						self.steering = -22

				# 모터 스피드 값 지정
				self.motor_msg.data = 2000

				# 스티어링 값 계산
				self.steering_command = (self.steering * self.servo_gain) + self.steering_angle_to_servo_offset
				self.servo_msg.data=self.steering_command

				# 스티어링 값 출력
				print(self.steering, self.steering_command)	

				# 모터 스피드, 스티어링 값 publish
				self.servo_pub.publish(self.servo_msg)
				self.motor_pub.publish(self.motor_msg)

			# 반복 실행 20hz 딜레이
			rate.sleep()

	# 영상 오프셋 메시지 받는 콜백 함수 
	def offset_callback(self,msg):
		self.is_offset = True
		self.offset = msg.data

# 메인 함수, Pure_pursuit 클래스 실행
if __name__ == '__main__':
	try:
		test_track = Pure_pursuit()
	except rospy.ROSInterruptException:
		pass