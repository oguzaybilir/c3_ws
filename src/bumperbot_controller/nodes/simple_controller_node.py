#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# kütüphaneleri import ediyoruz
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np

class SimpleController(object): # SimpleController adında bir sınıf oluşturuyoruz

    def __init__(self, wheel_radius, wheel_seperation): # sınıfa ait init fonksiyonunu oluşturuyoruz
        rospy.loginfo("Using wheel radius %d" % wheel_radius) # loginfo ile terminalde yazdırma yapıyoruz
        rospy.loginfo("Using wheel seperation %d" % wheel_seperation) # loginfo ile terminalde yazdırma yapıyoruz

        self.right_cmd_pub_ = rospy.Publisher("wheel_right_controller/command", Float64, queue_size=10) # wheel_right_controller/command topicine abone oluyoruz
        self.left_cmd_pub_ = rospy.Publisher("wheel_left_controller/command", Float64, queue_size=10) # wheel_left_controller/command topicine abone oluyoruz


        self.vel_sub = rospy.Subscriber("bumperbot_controller/cmd_vel", Twist, self.velCallback, queue_size=10) # bumperbot_controller/cmd_vel topicine abone oluyoruz

        self.speed_conversion_ = np.array([[wheel_radius / 2, wheel_radius / 2],
                                        [wheel_radius / wheel_seperation, -wheel_radius / wheel_seperation]]) # hız dönüşüm matrisini oluşturuyoruz


        rospy.loginfo("The conversion matrix is %s " % self.speed_conversion_ ) # loginfo ile terminalde yazdırma yapıyoruz


    def velCallback(self, msg): # velCallback adında bir fonksiyon oluşturuyoruz

        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]]) # robot hızını oluşturuyoruz
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) # tekerlek hızını hesaplıyoruz

        right_speed = Float64(wheel_speed[0,0]) # sağ tekerlek hızını oluşturuyoruz
        left_speed = Float64(wheel_speed[1,0]) # sol tekerlek hızını oluşturuyoruz

        self.right_cmd_pub_.publish(right_speed) # sağ tekerlek hızını yayınlıyoruz
        self.left_cmd_pub_.publish(left_speed) # sol tekerlek hızını yayınlıyoruz

if __name__ == "__main__": # main fonksiyonu oluşturuyoruz
    rospy.init_node("simple_controller") # simple_controller adında bir node oluşturuyoruz

    wheel_radius = rospy.get_param("~wheel_radius") # parametreleri alıyoruz
    wheel_seperation = rospy.get_param("~wheel_separation") # parametreleri alıyoruz
    controller = SimpleController(wheel_radius=wheel_radius, wheel_seperation=wheel_seperation) # SimpleController sınıfından bir nesne oluşturuyoruz
    
    rospy.spin() # ros'u döndürüyoruz
    