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

class make_path_tf :

    def __init__(self):
        rospy.init_node('make_path_tf', anonymous=True)

        listener = tf.TransformListener()
       

        self.path_pub = rospy.Publisher('/path',Path, queue_size=1)

        self.path_msg=Path()
        self.path_msg.header.frame_id='/map'

        self.prev_x=0
        self.prev_y=0

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('control_planning')
        full_path=pkg_path+'/path'+'/path.txt'
        self.f=open(full_path,'w')
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            

            waypint_pose=PoseStamped()
            
            x=trans[0]
            y=trans[1]
            distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
            if distance > 0.1 :
                waypint_pose.pose.position.x=x
                waypint_pose.pose.position.y=y
                waypint_pose.pose.orientation.w=1
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data='{0}\t{1}\n'.format(x,y)
                self.f.write(data) 
                self.prev_x=x
                self.prev_y=y
                print(x,y)
                


            rate.sleep()
        self.f.close()
  
            
        

if __name__ == '__main__':
    try:
        test_track=make_path_tf()
    except rospy.ROSInterruptException:
        pass

    