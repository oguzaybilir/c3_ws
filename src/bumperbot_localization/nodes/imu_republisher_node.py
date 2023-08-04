#!/usr/bin/env python3
#! -*- coding: UTF-8 -*-


import rospy # gerekli kütüphaneleri import et
from sensor_msgs.msg import Imu # sensör verisini al



def imuCallback(imu): # imu topiğine abone olan değişkenin gelen veriler ile ne yapacağına ait fonksiyonu oluştur
    imu.header.frame_id = "base_footprint_ekf" # değerlerin hangi frame üzerinden referans alınacağını belirliyoruz
    imu_pub_.publish(imu) # yaptığımız değişiklikliği publishliyoruz

if __name__ == "__main__": # eğer kod ok ise
    rospy.init_node("imu_republisher_node") # init_node oluşturuyoruz
    imu_pub_ = rospy.Publisher("imu_ekf", Imu, queue_size=10) # değişiklik yapılan imu verilerini publish etmek için bir publisher oluşturuyoruz
    imu_sub_ = rospy.Subscriber("imu", Imu, imuCallback) # imu topiğini dinliyoruz
    rospy.spin() # koda spin attırıyoruz