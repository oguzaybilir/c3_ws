#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# kütüphaneleri import ediyoruz
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf_conversions

from math import sin, cos

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleController(object): # SimpleController adında bir sınıf oluşturuyoruz

    def __init__(self, wheel_radius, wheel_seperation): # sınıfa ait init fonksiyonunu oluşturuyoruz
        rospy.loginfo("Using wheel radius %d" % wheel_radius) # loginfo ile terminalde yazdırma yapıyoruz
        rospy.loginfo("Using wheel seperation %d" % wheel_seperation) # loginfo ile terminalde yazdırma yapıyoruz

        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_seperation    
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ =  0.0
        self.prev_time_ = rospy.Time.now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"

        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"
        
        



        self.right_cmd_pub_ = rospy.Publisher("wheel_right_controller/command", Float64, queue_size=10) # wheel_right_controller/command topicine abone oluyoruz
        self.left_cmd_pub_ = rospy.Publisher("wheel_left_controller/command", Float64, queue_size=10) # wheel_left_controller/command topicine abone oluyoruz

        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom", Odometry, queue_size=10)


        self.vel_sub = rospy.Subscriber("bumperbot_controller/cmd_vel", Twist, self.velCallback, queue_size=10) # bumperbot_controller/cmd_vel topicine abone oluyoruz
        self.joint_sub_ = rospy.Subscriber("joint_states",JointState, self.jointCallback)


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

    ### TERSİNE KİNEMATİK FORMÜLLERİ İLE TEKERLERİN KONUMLARINI VE HIZLARINI ALIP İŞLİYORUZ
    def jointCallback(self, msg):

        dp_left = msg.position[0] - self.left_wheel_prev_pos_ # sol tekerin konum farkını alıyoruz
        dp_right = msg.position[1] - self.right_wheel_prev_pos_ # sağ tekerin konum farkını alıyoruz
        dt = (msg.header.stamp - self.prev_time_).to_sec() # delta zaman yani zamandaki değişimi, farkı alıyoruz ve saniye cinsine çeviriyoruz

        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = msg.header.stamp

        fi_left = dp_left / dt
        fi_right = dp_right / dt

        linear_ = (self.wheel_radius * fi_right + self.wheel_radius * fi_left) / 2 
        angular_ = (self.wheel_radius * fi_right - self.wheel_radius * fi_left) / self.wheel_separation

        d_s_ = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2 
        d_theta_ = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation

        self.theta_ += d_theta_
        self.x_ += d_s_ * cos(self.theta_)
        self.y_ += d_s_ * sin(self.theta_)

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear_
        self.odom_msg_.twist.twist.angular.z = angular_

        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = rospy.Time.now()

        self.br_.sendTransform(self.transform_stamped_)

        



if __name__ == "__main__": # main fonksiyonu oluşturuyoruz
    rospy.init_node("simple_controller") # simple_controller adında bir node oluşturuyoruz

    wheel_radius = rospy.get_param("~wheel_radius") # parametreleri alıyoruz
    wheel_seperation = rospy.get_param("~wheel_separation") # parametreleri alıyoruz
    controller = SimpleController(wheel_radius=wheel_radius, wheel_seperation=wheel_seperation) # SimpleController sınıfından bir nesne oluşturuyoruz
    
    rospy.spin() # ros'u döndürüyoruz
    