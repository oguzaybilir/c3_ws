// 

#include "bumperbot_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>
// #include <ros/ros.h>

SimpleController::SimpleController(const ros::NodeHandle &nh, double radius, double separation) : nh_(nh) // SimpleController sınıfının constructor fonksiyonunu oluşturuyoruz
{
    ROS_INFO_STREAM("Using wheel radius " << radius); // ROS_INFO_STREAM fonksiyonu ile terminalde "Using wheel radius " yazdırıyoruz ve ardından radius değişkenini yazdırıyoruz
    ROS_INFO_STREAM("Using wheel separation " << separation); // ROS_INFO_STREAM fonksiyonu ile terminalde "Using wheel separation " yazdırıyoruz ve ardından separation değişkenini yazdırıyoruz

    right_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_controller/command", 10); // nh_ değişkeninden wheel_right_controller/command topic'ine 10 mesaj gönderilebileceğini belirtiyoruz
    left_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_controller/command", 10); // nh_ değişkeninden wheel_left_controller/command topic'ine 10 mesaj gönderilebileceğini belirtiyoruz

    vel_sub_ = nh_.subscribe("bumperbot_controller/cmd_vel", 1000, &SimpleController::velCallback, this); // nh_ değişkeninden bumperbot_controller/cmd_vel topic'inden 1000 mesaj alabileceğimizi belirtiyoruz ve velCallback fonksiyonunu çağırıyoruz

    speed_conversion_ << radius / 2, radius / 2, radius / separation, -radius / separation; // speed_conversion_ değişkenine matris atıyoruz

    ROS_INFO_STREAM("The conversion matrix is \n" << speed_conversion_); // ROS_INFO_STREAM fonksiyonu ile terminalde "The conversion matrix is " yazdırıyoruz ve ardından speed_conversion_ değişkenini yazdırıyoruz
}


void SimpleController::velCallback(const geometry_msgs::Twist &msg) // velCallback fonksiyonunu oluşturuyoruz
{
    Eigen::Vector2d robot_speed(msg.linear.x, msg.angular.z); // Eigen kütüphanesinden Vector2d sınıfından robot_speed değişkeni oluşturuyoruz ve msg.linear.x ve msg.angular.z değişkenlerini atıyoruz
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed; // wheel_speed değişkenine speed_conversion_ değişkeninin tersini alıyoruz ve robot_speed değişkenini çarpıyoruz

    std_msgs::Float64 right_speed; // std_msgs kütüphanesinden Float64 sınıfından right_speed değişkeni oluşturuyoruz
    std_msgs::Float64 left_speed; // std_msgs kütüphanesinden Float64 sınıfından left_speed değişkeni oluşturuyoruz

    right_speed.data = wheel_speed.coeff(0); // right_speed değişkenine wheel_speed değişkeninin 0. katsayısını atıyoruz
    left_speed.data = wheel_speed.coeff(1); // left_speed değişkenine wheel_speed değişkeninin 1. katsayısını atıyoruz

    right_cmd_pub_.publish(right_speed); // sağ hızı publish ediyoruz
    left_cmd_pub_.publish(left_speed); // sol hızı publish ediyoruz

}
