#!/usr/bin/env python3
#! -*- coding: UTF-8 -*-


import rospy
from sensor_msgs.msg import Imu



def imuCallback(imu):
    imu.header.frame_id = "base_footprint_ekf"
    imu_pub_.publish(imu)
    
if __name__ == "__main__":
    rospy.init_node("imu_republisher_node")
    imu_pub_ = rospy.Publisher("imu_ekf", Imu, queue_size=10)
    imu_sub_ = rospy.Subscriber("imu", Imu, imuCallback)
    rospy.spin()