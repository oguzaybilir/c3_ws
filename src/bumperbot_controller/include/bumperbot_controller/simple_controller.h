// SimpleController sınıfımızın header dosyasını oluşturuyoruz
#ifndef SIMPLE_CONTROLLER_H // eğer SIMPLE_CONTROLLER_H tanımlı değilse
#define SIMPLE_CONTROLLER_H // SIMPLE_CONTROLLER_H tanımlı olsun

// kütüphanelerimizi import ediyoruz
#include <ros/ros.h> // ana ros kütüphanesi
#include <geometry_msgs/Twist.h> // hız dediğimiz zaman akla gelen ilk sınıf geometry_msgs paketindeki Twist sınıfıdır
#include <sensor_msgs/JointState.h> // Robotun linklerinden biri olan tekerimize ait joint(eklem) bilgilerine (bu bilgiler belirli sensörler tarafından alınan
// bilgiler) erişmek için alıp JointState sınıfını çağırıyoruz
#include <nav_msgs/Odometry.h> // Odometry verilerini almak için nav_msgs paketinden Odometry sınıfını çağırıyoruz 
#include <Eigen/Core> // eigen = öz vekör (lineer cebir)
#include <geometry_msgs/TransformStamped.h> // zamana bağlı bir TF (transform, dönüşüm) yapıldığı için geometry_msgs paketinden TransformStamped sınıfını çağırıyoruz


class SimpleController // SimpleController sınıfımızı oluşturuyoruz
{
public: // public kısmı
    SimpleController(const ros::NodeHandle &, double radius, double separation); // public kısmına constructor fonksiyonumuzu tanımlıyoruz

private: // private kısmı
    void velCallback(const geometry_msgs::Twist &); // private kısmına velCallback fonksiyonumuzu tanımlıyoruz

    void jointCallback(const sensor_msgs::JointState &);

    ros::NodeHandle nh_; // ros node handle oluşturuyoruz ve nh_ değişkenine atıyoruz
    ros::Subscriber vel_sub_; // ros subscriber oluşturuyoruz ve vel_sub_ değişkenine atıyoruz
    ros::Subscriber joint_sub_;
    ros::Publisher right_cmd_pub_; // ros publisher oluşturuyoruz ve right_cmd_pub_ değişkenine atıyoruz
    ros::Publisher left_cmd_pub_; // ros publisher oluşturuyoruz ve left_cmd_pub_ değişkenine atıyoruz
    ros::Publisher odom_pub_;
    Eigen::Matrix2d speed_conversion_; // Eigen kütüphanesinden Matrix2d sınıfından speed_conversion_ değişkeni oluşturuyoruz

    // araca ait kinematik ve tersine kinematik formüllerinde bulunan değişkenleri (bu değişkenler araca ait özellikleri tutuyorlar) oluşturuyoruz
    double wheel_radius_; 
    double wheel_separation_;

    // transformlarda ve odom hesabında kullanılacak olan değişkenleri oluşturuyoruz
    double left_wheel_prev_pos_; 
    double right_wheel_prev_pos_;
    ros::Time prev_time_;

    double x_;
    double y_;
    double theta_;

    nav_msgs::Odometry odom_msg_; // Odometry sınıfına ait bir odom_msg_ objesi oluşturuyoruz, bu obje ile Odometry verilerine erişebilecek ve değiştirebileceğiz

    geometry_msgs::TransformStamped transform_stamped_; // Yapılacak olan transformlar için TransformStamped sınıfına ait bir transform_stamped_ objesi oluşturuyoruz
    
    


};
#endif // SIMPLE_CONTROLLER_H