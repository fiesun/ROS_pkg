#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from math import cos, sin, pi
from geometry_msgs.msg import Point32


class Parking_guardrail:
    def __init__(self):
        # 주차 노드 생성
        rospy.init_node('Parking_guardrail', anonymous=True)

        # 라이다 값 받는 Subscriber
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # 주차 알고리즘 실행 시 모터 스피드, 스티어링 값 Publisher
        self.motor_pub = rospy.Publisher('/pk/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/pk/commands/servo/position', Float64, queue_size=1)

        # 라이다 값 Publisher
        self.pcd_pub = rospy.Publisher('laser2pcd', PointCloud, queue_size=1)
        # 주차 상태 여부 Publisher
        self.park_pub = rospy.Publisher('/park/state', Float64, queue_size=1)

        # 센서 값 상태 초기화
        self.is_sensor = False
        # 주차 실행 시 시간 측정 실행 하기 위한 변수
        self.is_check = False
        # 주차 실행 시 소요 시간 받는 변수 초기화
        self.parking_second = 0
        # 주차 알고리즘 상태 초기화
        self.state = 0

        # ROS 30hz 지연
        rate = rospy.Rate(30)
        # 반복 실행
        while not rospy.is_shutdown():
            # 반복 실행 30hz 딜레이
            rate.sleep()

    # 라이다 메시지 값 받는 콜백 함수
    def laser_callback(self, msg):
        pcd = PointCloud()    
        motor_msg = Float64()  
        servo_msg = Float64()
        park_msg = Float64()
        pcd.header.frame_id = msg.header.frame_id

        # 라이다 포인트 앵글 초기화
        angle = 0

        # 라이다 포인트 x, y 값 전달 
        for r in msg.ranges:
            tmp_point = Point32()
            tmp_point.x = r * cos(angle)
            tmp_point.y = r * sin(angle)

            angle = angle + (1.0 / 180 * pi)

            if r < 12:
                pcd.points.append(tmp_point)

        # 라이다 포인트 갯수 초기화
        count_x = 0
        count_y = 0

        # 전방 2미터 이내 x 포인트 개수 count_x, 우측 방향 전체 y 포인트 개수 count_y
        for point in pcd.points:
            if point.x > 0 and point.x < 2:
                count_x += 1
            elif point.y > 0:
                count_y += 1

        # ros 현재 시간 받아오는 변수
        self.second = rospy.get_time()

        # 처음 상태에서 라이다 포인트 x 개수가 55 이상이 될 때까지 우측 방향으로 이동
        if self.sate == 0:
            servo_msg.data = 0.85
            motor_msg.data = 800
            # 포인트 x 개수 55 이상일 때 상태 1로 변환
            if count_x >= 55:
                self.sate = 1

        # 상태 1일 때 라이다 포인트 x 값 55 이하 될 때까지 좌측 방향 이동 (가드레일과 평행 유지)
        elif self.sate == 1:
            servo_msg.data = 0.15
            motor_msg.data = 800
            # 포인트 x 개수 55 이하일 때 상태 2로 변환
            if count_x <= 55:
                self.sate = 2

        # 상태 2 일 때 (가드레일 평행 유지) 스티어링 전방으로 회전
        elif self.sate == 2:
            servo_msg.data = 0.5304
            motor_msg.data = 0
            # 주차 경과 시간 측정
            if self.is_check == False:
                self.is_check = True
                self.parking_second = rospy.get_time()
            # 주차 5초 경과 시 상태 3으로 변환
            if self.parking_second + 5 < self.second:
                self.sate = 3

        # 상태 3 일 때 포인트 y 값 100 이상 될 때까지 좌측 방향 이동(라인 복귀)
        elif self.sate == 3:
            servo_msg.data = 0.15
            motor_msg.data = 800
            # 포인트 y 값 100 이상 시 상태 4로 변환
            if count_y >= 100:
                self.s = 4

        # 상태 4일 때 라이다 포인트 x 값 23 이상 될 때 까지 우측 방향 이동(라인과 평행 유지)
        elif self.sate == 4:
            servo_msg.data = 0.85
            motor_msg.data = 800
            # 라이다 포인트 x 값 23 이상 시(라인과 평행 유지) 상태 5로 변환
            if count_x >= 23:
                self.sate = 5
                
        # 상태 5일 때 스티어링 전방으로 회전
        elif self.sate == 5:
            servo_msg.data = 0.5304
            motor_msg.data = 0
            # 주차 알고리즘 끝날 시 1로 변환
            park_msg = 1
            # 주차 상태 메시지 publish
            self.park_pub.publish(park_msg)

        # 포인트 개수, 상태 확인 시 필요한 print
        print(count_x, count_y, self.sate)

        # 모터 스피드, 스티어랑 값 publish
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)
        # 라이다 포인트 값 publish
        self.pcd_pub.publish(pcd)

# 메인 함수, Parking_guardrail 클래스 실행
if __name__ == '__main__':
    try:
        test_track = Parking_guardrail()
    except rospy.ROSInterruptException:
        pass