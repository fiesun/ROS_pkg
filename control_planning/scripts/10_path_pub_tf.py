#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class path_pub_tf :

    def __init__(self):
        rospy.init_node('path_pub_tf', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/path',Path, queue_size=1)
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='/map'
        

        self.local_path_size=20

        listener = tf.TransformListener()

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('control_planning')
        full_path=pkg_path+'/path'+'/path.txt'
        self.f=open(full_path,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            
            x=trans[0]
            y=trans[1]
            min_dis=float('inf')
            current_waypoint=-1
            for i,waypoint in enumerate(self.global_path_msg.poses) :

                distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                if distance < min_dis :
                    min_dis=distance
                    current_waypoint=i

            
            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1
                        local_path_msg.poses.append(tmp_pose)
                
                else :
                    for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1
                        local_path_msg.poses.append(tmp_pose)


            print(x,y)
            self.global_path_pub.publish(self.global_path_msg)
            self.local_path_pub.publish(local_path_msg)
            rate.sleep()
        
        



if __name__ == '__main__':
    try:
        test_track=path_pub_tf()
    except rospy.ROSInterruptException:
        pass