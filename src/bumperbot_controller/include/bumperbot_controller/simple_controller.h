// SimpleController sınıfımızın header dosyasını oluşturuyoruz
#ifndef SIMPLE_CONTROLLER_H // eğer SIMPLE_CONTROLLER_H tanımlı değilse
#define SIMPLE_CONTROLLER_H // SIMPLE_CONTROLLER_H tanımlı olsun

// kütüphanelerimizi import ediyoruz
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>

class SimpleController // SimpleController sınıfımızı oluşturuyoruz
{
public: // public kısmı
    SimpleController(const ros::NodeHandle &, double radius, double separation); // public kısmına constructor fonksiyonumuzu tanımlıyoruz

private: // private kısmı
    void velCallback(const geometry_msgs::Twist &); // private kısmına velCallback fonksiyonumuzu tanımlıyoruz

    ros::NodeHandle nh_; // ros node handle oluşturuyoruz ve nh_ değişkenine atıyoruz
    ros::Subscriber vel_sub_; // ros subscriber oluşturuyoruz ve vel_sub_ değişkenine atıyoruz
    ros::Publisher right_cmd_pub_; // ros publisher oluşturuyoruz ve right_cmd_pub_ değişkenine atıyoruz
    ros::Publisher left_cmd_pub_; // ros publisher oluşturuyoruz ve left_cmd_pub_ değişkenine atıyoruz
    Eigen::Matrix2d speed_conversion_; // Eigen kütüphanesinden Matrix2d sınıfından speed_conversion_ değişkeni oluşturuyoruz
};

#endif // SIMPLE_CONTROLLER_H