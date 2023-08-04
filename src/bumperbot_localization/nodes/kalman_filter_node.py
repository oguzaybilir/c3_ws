#!/usr/bin/env python3
#! -*- coding: UTF-8 -*-

import rospy # gerekli kütüphaneleri import ediyoruz
from nav_msgs.msg import Odometry # nav_msgs paketinden Odometry sınıfını çağırıyoruz
from sensor_msgs.msg import Imu # sensor_msgs paketinden Imu sınıfını çağırıyoruz

class KalmanFilter(object): # KalmanFilter adında bir sınıf oluşturuyoruz

    def __init__(self): # init fonksiyonunu oluşturuyoruz

        self.odom_sub_ = rospy.Subscriber("bumperbot_controller/odom_noisy", Odometry, self.odomCallback) # odom_noisy topiğine abone oluyoruz
        self.imu_sub_ = rospy.Subscriber("imu", Imu, self.imuCallback) # imu topiğine abone oluyoruz
        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom_kalman", Odometry, queue_size=10) # kalman filtresi ile düzenlenen odom verilerini publish edecek olan publisher'ı oluşturuyoruz

        # init değerleri atıyoruz
        self.mean_ = 0.0 # dağılımın ortalama değeri
        self.variance_ = 1000.0 # dağılımın standart sapmasının karesi
        self.imu_angular_z = 0.0 # imudan gelen açısal dönüş değeri
        self.is_first_odom_ = True # ilk odom frame'i mi değil mi bool kontrolcüsü
        self.last_angular_z_ = 0.0 # son açısal dönüş değerini bir değişkene atıyoruz
        self.motion_ = 0.0 # hareket değerini atıyoruz
        self.kalman_odom_ = Odometry() # Odometry tipinde bir değişken oluşturuyoruz
        self.motion_variance_ = 4.0 # hareket varyansını atıyoruz
        self.measurement_variance_ = 0.5 # ölçüm varyansını atıyoruz

    def odomCallback(self, odom): # odom_sub_ subscriber'ının dinlediği veriler ile ne yapacağına dair bir fonk yazıyoruz
        self.kalman_odom_ = odom # self.kalman_odom_ değişkenini callback fonksiyonuna gelen veriye eşitliyoruz
        
        if self.is_first_odom_: # eğer self.is_first_odom_ True ise
            self.mean_ = odom.twist.twist.angular.z # mean_ değerini gelen verinin açısal dönüş değerine eşitle
            self.last_angular_z_ = odom.twist.twist.angular.z # son açısal dönüş değerini şuan gelen veriye eşitle
            self.is_first_odom_ = False # is first_odom değerini False yap
            return
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_ # hareket değerini güncelliyoruz
        
        self.statePrediction() # self.statePrediction() fonksiyonunu çalıştırıyoruz
        self.measurementUpdate() # self.measurementUpdate() fonksiyonunu çalıştırıyoruz

        self.kalman_odom_.twist.twist.angular.z =  self.mean_ # kalman_odom_'a ait açısal dönüş elemanını self.mean_'e eşitliyoruz
        self.odom_pub_.publish(self.kalman_odom_) # self.kalman_odom_ güncellendi, artık yayınlayabiliriz
        self.last_angular_z_ = odom.twist.twist.angular.z # son açısal dönüş değerini şuanki değere eşitliyoruz ki diğer iterasyonda last_angular_z_ doğru olsun ve hesaplamada hata olmasın
        

    def imuCallback(self, imu): # imu topiğini dinleyen subscriber'ın aldığı veriler ile yapacağı işlemi yazıyoruz
        self.imu_angular_z_ = imu.angular_velocity.z # self.imu_angular_z_ değişkenini sensörden gelen değere eşitliyoruz

    def measurementUpdate(self): # kalman filtresinin ikinci adımı olan fonksiyonu yazıyoruz
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z) / (self.variance_ + self.measurement_variance_) # self.mean_ değerini güncelliyoruz
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_) # self.variance_ değerini güncelliyoruz

    def statePrediction(self): # kalman filtresinin üçüncü adımı
        self.mean_ = self.mean_ + self.motion_ # yeni hareket değerini bulup mean_'e eşitliyoruz
        self.variance_ = self.variance_ + self.motion_variance_ # yeni varyans değerini hesaplayıp variance_'a eşitliyoruz

        

if __name__ == "__main__": # eğer dosya ok ise
    rospy.init_node("kalman_filter_node") # init_node oluştur ve ismini kalman_filter_node yap
    filter = KalmanFilter() # KalmanFilter tipinde bir filter constructor elemanı oluştur
    rospy.spin() # kodun sürekli çalışmasını sağla