#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi
from geometry_msgs.msg import Point32
import tf


class simple_controller :

    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
     

        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        



        
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():

            

            rate.sleep()


    def laser_callback(self,msg):
        pcd=PointCloud()
        motor_msg=Float64()
        servo_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id

        angle=0
        for r in msg.ranges :

            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            angle=angle+(1.0/180*pi)
            if r<12  :
                pcd.points.append(tmp_point)

        area_size=15
        area=[[90,75], #90~76
              [75,60], #75~61
              [60,45], #60~46
              [45,30], #45~31
              [30,15], #30~16
              [15,0],  #15~1
              [359,344], # 359 ~ 345
              [344,329], # 344 ~ 330
              [329,314], # 329 ~ 315
              [314,299], # 314 ~ 300
              [299,284], # 299 ~ 285
              [284,269]] # 284 ~ 270
        avg=[]

        for area_num in range(len(area)):
            sum=0
            for degree in range(area[area_num][0],area[area_num][1],-1) :
                r=msg.ranges[degree]
                if r> 12 :
                    r=12
                sum=sum+r
            
            avg.append(sum/area_size)
            print(area_num,avg[area_num])

        
        avg_max=max(avg)
        avg_max_index=avg.index(max(avg))
        print(avg_max_index,avg_max)


        

        if avg_max_index == 0 or avg_max_index == 1 or avg_max_index==2 :
            servo_msg.data=0.15
        elif avg_max_index == 3 or avg_max_index == 4 :
            servo_msg.data=0.35
        elif avg_max_index == 5 or avg_max_index ==6 :
            servo_msg.data=0.5304
        elif avg_max_index == 7 or avg_max_index ==8 :
            servo_msg.data=0.72
        elif avg_max_index == 9 or avg_max_index == 10 or avg_max_index==11 :
            servo_msg.data=0.85
        
        

        
    


        motor_msg.data=1500
        
        self.motor_pub.publish(motor_msg)
        self.pcd_pub.publish(pcd)
        self.servo_pub.publish(servo_msg)







if __name__ == '__main__':
    try:
        test_track=simple_controller()
    except rospy.ROSInterruptException:
        pass