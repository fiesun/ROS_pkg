#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Float32


class Process:
    def __init__(self):
        # 프로세스 노드 생성
        rospy.init_node('Process', anonymous=True)

        # 충격감지센서 데이터 값 받아오는 Subscriber
        rospy.Subscriber("/chatter", Float32, self.chatter_callback)

        # 주차 알고리즘 실행 시 모터 스피드, 스티어링 값 받아오는 Subscriber
        rospy.Subscriber("/pk/commands/motor/speed", Float64, self.pk_speed_callback)             
        rospy.Subscriber("/pk/commands/servo/position", Float64, self.pk_position_callback)

        # 라인 주행 시 모터 스피드, 스티어링 값 받아오는 Subscriber                 
        rospy.Subscriber("/ln/commands/motor/speed", Float64, self.ln_speed_callback)        
        rospy.Subscriber("/ln/commands/servo/position", Float64, self.ln_position_callback)

        # 주차 상태 여부 받아오는 Subscriber
        rospy.Subscriber("/park/state", Float64, self.park_callback)

        # RC카에 모터 스피드, 스티어링 값 전달해주는 Publisher
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

        # 충격 감지 센서 값 초기화
        self.is_sensor = False

        # 라인 주행 시 필요한 모터 스피드, 스티어링 값 초기화
        self.ln_speed_value = 0
        self.ln_position_value = 0

        # 주차 알고리즘 싱행 시 필요한 모터 스피드, 스티어링 값 초기화
        self.pk_speed_value = 0
        self.pk_position_value = 0

        # 주차 상태 초기화
        self.park_state = 0

        # 모터, 스티어링 데이터 타입 지정
        motor_msg = Float64()
        servo_msg = Float64()

        # ROS 30hz 지연
        rate = rospy.Rate(30)

    # 충격감지 센서 상태 초기화
	self.sensor_state = 0

        # 전체 프로세스 반복 실행
        while not rospy.is_shutdown():
            # 충격감지 센서 감지 여부 확인
            if self.is_sensor == True:
                # 모터 스피드, 스티어링 값 전달
                self.motor_pub.publish(motor_msg)
                self.servo_pub.publish(servo_msg)
                # 충격감지 센서 미 감지 혹은 주차 상태 일 때 라인 주행 모터 스피드, 스티어링 값 전달
                if self.sensor_state == 0 or self.park_state == 1:
                    motor_msg = self.ln_speed_value
                    servo_msg = self.ln_position_value
                # 충격감지 센서 감지 시 주차 알고리즘 모터 스피드, 스티어링 값 전달
                elif self.sensor_state == 1:
                    motor_msg = self.pk_speed_value
                    servo_msg = self.pk_position_value
            # 반복 실행 딜레이 30hz
            rate.sleep()

    # 주차 상태 메시지 받는 콜백 함수
    def park_callback(self, msg):
        self.park_state = msg.data

    # 충격감지 센서 메시지 받는 콜백 함수
    def chatter_callback(self, msg):
        self.is_sensor = True
        self.sensor_value = msg.data
    # 센서 값 1000 이하 시 상태 센서 상태 1
	if self.sensor_value <= 1000:
	    self.sensor_state = 1

    # 주차 알고리즘에 해당하는 모터 스피드, 스티어링 메시지 값 받는 콜백 함수
    def pk_speed_callback(self, msg):
        self.pk_speed_value = msg.data
    def pk_position_callback(self, msg):
        self.pk_position_value = msg.data

    # 라인 주행 알고리즘에 해당하는 모터 스피드, 스티어링 메시지 값 받는 콜백 함수
    def ln_speed_callback(self, msg):
        self.ln_speed_value = msg.data
    def ln_position_callback(self, msg):
        self.ln_position_value = msg.data

# 메인 함수, Process 클래스 실행
if __name__ == '__main__':
    try:
        test_track = Process()
    except rospy.ROSInterruptException:
        pass