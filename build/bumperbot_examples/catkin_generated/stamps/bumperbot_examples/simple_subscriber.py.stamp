#!/usr/bin/env python3
#-*- coding:UTF-8 -*-

# kütüphaneleri import ediyoruz
import rospy
from std_msgs.msg import String



def msgCallback(msg): # mesaj geldiğinde çalışacak fonksiyon
    rospy.loginfo("New message received %s", msg.data) # mesajı loginfo ile yazdırıyoruz


if __name__ == "__main__": # eğer bu dosya main dosya ise

    rospy.init_node("simple_subscriber_py",anonymous=True) # init_node tanımlaması yapıyoruz ve isimlendiriyoruz
    rospy.Subscriber("chatter",String, msgCallback) #   subscriber oluşturuyoruz

    rospy.spin() # rospy kapanana kadar döngüyü çalıştırıyoruz

    

