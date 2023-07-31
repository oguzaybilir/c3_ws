#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# kütüphaneleri import ediyoruz
import rospy # Ana ros kütüphanesi
from std_msgs.msg import Float64 # Aracımızın sağ ve sol tekerine ait hız kontrolcülerinden yayınlanacak olan mesajlar Float64 tipindeki sayısal mesajlardır
from geometry_msgs.msg import Twist # Hız dediğimizde akla gelen ilk şey Twist mesaj tipidir, Twist hız mesajını dinlediğimiz topiktir
import numpy as np # arraylar ile işlem yaparken kullandığımız kütüphane
from sensor_msgs.msg import JointState # Aracın tekerlerini konumu alırken dinlediğimiz topiktir
from nav_msgs.msg import Odometry # robotun Odometry verilerini alırken dinlediğimiz topiktir
import tf_conversions # euler to quaternion dönüşümü yaparken kullandığımız kütüphanedir

from math import sin, cos # normal sin ve cos 

from tf2_ros import TransformBroadcaster # tf2_ros kütüphanesini kullanarak yaptığımız transformları yayınlamak için kullandığımız sınıftır
from geometry_msgs.msg import TransformStamped # Zamana bağlı dönüşümler yapıldığı için geometry_msgs paketinden TransformStamped sınıfını çağırıyoruz

class SimpleController(object): # SimpleController adında bir sınıf oluşturuyoruz

    def __init__(self, wheel_radius, wheel_seperation): # sınıfa ait init fonksiyonunu oluşturuyoruz
        rospy.loginfo("Using wheel radius %d" % wheel_radius) # loginfo ile terminalde yazdırma yapıyoruz
        rospy.loginfo("Using wheel seperation %d" % wheel_seperation) # loginfo ile terminalde yazdırma yapıyoruz

        # Robota ait kinematik, tersine kinematik ve odometry hesaplamalarında kullanılacak olan init değişkenleri tanımlıyoruz
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_seperation    
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ =  0.0
        self.prev_time_ = rospy.Time.now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Odometry 
        self.odom_msg_ = Odometry() # Odometry sınıfına ait bir öge oluşturduk ve bu ögeyi self.odom_msg_ değişkenine atadık
        
        # Hangi frame den hangi frame e Odometry hesaplanacağını self.odom_msg_ objesine ait mesajlara bildirdik
        self.odom_msg_.header.frame_id = "odom" 
        self.odom_msg_.child_frame_id = "base_footprint"

        # self.odom_msg_ objesine ait orientation(yön) değerlerini init olarak atadık
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster() # Yapılan transformları yayınlaması için TransformBroadcaster tipinde bir self.br_ objesi oluşturduk
        self.transform_stamped_ = TransformStamped() # Transformlar zamana bağlı yapılacağı için TransformStamped() tipinde bir self.transform_stamped_ objesi oluşturduk

        # Hangi frame'den hangi frame'e dönüşüm yapılacağını self.transform_stamped_ objesine ait mesajlara bildirdik
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.right_cmd_pub_ = rospy.Publisher("wheel_right_controller/command", Float64, queue_size=10) # wheel_right_controller/command topicine abone oluyoruz
        self.left_cmd_pub_ = rospy.Publisher("wheel_left_controller/command", Float64, queue_size=10) # wheel_left_controller/command topicine abone oluyoruz

        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom", Odometry, queue_size=10) # Odometry verilerimizi yayınlamak için bir publisher node oluşturuyoruz


        self.vel_sub = rospy.Subscriber("bumperbot_controller/cmd_vel", Twist, self.velCallback, queue_size=10) # bumperbot_controller/cmd_vel topicine abone oluyoruz
        self.joint_sub_ = rospy.Subscriber("joint_states",JointState, self.jointCallback) # Robotun tekerlerine ait joint(eklem) konumlarını almak için JointStates topiğine abone oluyoruz


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

        self.left_wheel_prev_pos_ = msg.position[0] # Robotun sol tekerinin anlık konum bilgisini self.left_wheel_prev_pos_ değişkenine atıyoruz
        self.right_wheel_prev_pos_ = msg.position[1] # Robotun sağ tekerinin anlık konum bilgisi self.right_wheel_prev_pos_ değişkenine atıyoruz
        self.prev_time_ = msg.header.stamp # JointStates topiğinden gelen mesajdaki zaman bilgisini self.prev_time_ değişkenine atıyoruz

        fi_left = dp_left / dt # sol tekerin dönüş hızını hesaplamak için sol tekerin katettiği yol / geçen zaman formulünü kullanıyoruz
        fi_right = dp_right / dt # sağ tekerin dönüş hızını hesaplamak için sağ tekerin katettiği yol / geçen zaman formülünü kullanıyoruz

        linear_ = (self.wheel_radius * fi_right + self.wheel_radius * fi_left) / 2  # Robotun lineer hızını hesaplıyoruz
        angular_ = (self.wheel_radius * fi_right - self.wheel_radius * fi_left) / self.wheel_separation # Robotun açısal hızını hesaplıyoruz

        d_s_ = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2 # Robotun konumundaki değişimi hesaplıyoruz
        d_theta_ = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation # Robotun açısındaki değişimi hesaplıyoruz

        self.theta_ += d_theta_ # güncel açı değeri ile açıdaki değişimi toplayarak yeni açı değerini elde ediyoruz
        self.x_ += d_s_ * cos(self.theta_) # yeni x konumunu bulmak için aracın konumundaki değişim * yeni açı değerinin kosinüsünü çarpıyoruz
        self.y_ += d_s_ * sin(self.theta_) # yeni y konumunu bulmak için aracın konumundaki değişim * yeni açı değerinin sinüsünü çarpıyoruz


        # Quaternion 

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta_) # euler açılarından quaternion matrisi oluşturuyoruz

        # self.odom_msg_ değişkeninin ORIENTATION verileri Quaternion matrisinden aldığımız veriler ile değiştiriyoruz
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = rospy.Time.now() # zamana bağlı işlem yapıldığı için şimdiki zamanı alıyoruz

        # self.odom_msg_ değişkeninin position ve twist verilerini yukarıda güncellenen self.x_ , self.y_ , lineer hız ve açısal hız verileri ile değiştiriyoruz
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear_
        self.odom_msg_.twist.twist.angular.z = angular_

        self.odom_pub_.publish(self.odom_msg_) # self.odom_msg_ mesajımızı doldurduğumuza göre artık publish edebiliriz

        # Transform

        # Robotun dönüşümde kullanılacak olan konum bilgilerini yukarıda güncellenen self.x_ ve self.y_ ile değiştirdik
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_

        # Robotun transformda kullanılacak olan ROTATION verilerini QUATERNION'dan gelen veriler ile güncelliyoruz
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = rospy.Time.now()

        self.br_.sendTransform(self.transform_stamped_) # Transformlar tamamlandığına göre artık dönüşümlerimizi broadcaster'a gönderebiliriz



if __name__ == "__main__": # main fonksiyonu oluşturuyoruz
    rospy.init_node("simple_controller") # simple_controller adında bir node oluşturuyoruz

    wheel_radius = rospy.get_param("~wheel_radius") # parametreleri alıyoruz
    wheel_seperation = rospy.get_param("~wheel_separation") # parametreleri alıyoruz
    controller = SimpleController(wheel_radius=wheel_radius, wheel_seperation=wheel_seperation) # SimpleController sınıfından bir nesne oluşturuyoruz
    
    rospy.spin() # ros'u döndürüyoruz
    