#!/usr/bin/env python3
#! -*- coding: utf-8 -*-

# Gerekli kütüphaneleri import ediyoruz.
import rospy
from bumperbot_examples.srv import AddTwoInts, AddTwoIntsResponse # Servis mesajımızı import ediyoruz
import sys # Bu kütüphaneyi terminalden argüman alacağımız için kullanıyoruz.

if __name__ == "__main__": # Başlangıç koşulu olarak bir python idiom'u olan if __name__ == "__main__": kullanıyoruz.
    if len(sys.argv) == 3: # Eğer terminalden gelen değerlerin sayısı 3'e eşitse
        a = int(sys.argv[1]) # a'yı argüman vektörünün 1. index'ine (yani ikinci elemanına) eşitliyoruz.
        b = int(sys.argv[2]) # b'yi argüman vektörünün 2. index'ine (yani üçüncü elemanına) eşitliyoruz.
        print("Requesing ", a, b) 
        rospy.wait_for_service("add_two_ints") # burada servera verileri gönderdik ve dönüş bekliyoruz, eğer dönüş alamazsak kod sonsuza kadar bekleyecek.
        add_two_ints = rospy.ServiceProxy("add_two_ints", AddTwoInts) # ServiceProxy ile servis adımızı ve .srv servis elemanımızı veriyoruz.
        # bu sayede servisle client arasındaki bağlantıyı kuruyoruz. 
        response = add_two_ints(a, b) # response değişkenine ServiceProxy'e bağlı olan add_two_ints()'i eşitliyoruz.
        # add_two_ints() bize response mesajını döndürüyor.
        print("Service response", response) # cevabı ekrana yazdırıyoruz.

    else: # Eğer terminalden gelen değerlerin sayısı 3'e eşit değilse
        print("Requested two arguments") # İki argüman bekliyoruz diye bir yazıyı ekrana yazdır.
        sys.exit(-1) # sys.exit(-1) demek kodu kapat demektir.
