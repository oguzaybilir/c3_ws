#!/usr/bin/env python3
#-*- coding:UTF-8 -*-

# kütüphaneleri import ediyoruz 
import rospy
from std_msgs.msg import String

# main fonksiyonu
if __name__ == '__main__': # eğer bu dosya main dosya ise
    rospy.init_node("simple_publisher_py",anonymous=True) # init_node tanımlaması yapıyoruz ve isimlendiriyoruz
    pub = rospy.Publisher("chatter",String, queue_size=10) # publisher oluşturuyoruz
    r = rospy.Rate(10) # 10 hz de çalışacak şekilde rate oluşturuyoruz
    counter = 0 # counter değişkeni oluşturuyoruz

    while not rospy.is_shutdown(): # rospy kapanana kadar döngüyü çalıştırıyoruz
        hello_msg = "Hello World from Python : %d" % counter # mesajımızı oluşturuyoruz
        pub.publish(hello_msg) # mesajımızı yayınlıyoruz

        r.sleep() # sistemi rate miktarı kadar süre uykuya sokuyoruz
        counter += 1 # counter değerini arttırıyoruz



