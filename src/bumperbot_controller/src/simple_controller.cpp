
#include "bumperbot_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

SimpleController::SimpleController(const ros::NodeHandle &nh, double radius, double separation) :
            nh_(nh),
            wheel_radius_(radius),
            wheel_separation_(separation),
            left_wheel_prev_pos_(0.0),
            right_wheel_prev_pos_(0.0),
            x_(0.0),
            y_(0.0),
            theta_(0.0) // SimpleController sınıfının constructor fonksiyonunu oluşturuyoruz
{
    ROS_INFO_STREAM("Using wheel radius " << radius); // ROS_INFO_STREAM fonksiyonu ile terminalde "Using wheel radius " yazdırıyoruz ve ardından radius değişkenini yazdırıyoruz
    ROS_INFO_STREAM("Using wheel separation " << separation); // ROS_INFO_STREAM fonksiyonu ile terminalde "Using wheel separation " yazdırıyoruz ve ardından separation değişkenini yazdırıyoruz


    right_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_controller/command", 10); // nh_ değişkeninden wheel_right_controller/command topic'ine 10 mesaj gönderilebileceğini belirtiyoruz
    left_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_controller/command", 10); // nh_ değişkeninden wheel_left_controller/command topic'ine 10 mesaj gönderilebileceğini belirtiyoruz

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("bumperbot_controller/odom", 10);
    vel_sub_ = nh_.subscribe("bumperbot_controller/cmd_vel", 1000, &SimpleController::velCallback, this); // nh_ değişkeninden bumperbot_controller/cmd_vel topic'inden 1000 mesaj alabileceğimizi belirtiyoruz ve velCallback fonksiyonunu çağırıyoruz

    joint_sub_ = nh_.subscribe("joint_states", 1000, &SimpleController::jointCallback, this);

    speed_conversion_ << radius/2, radius/2, radius/separation, -radius/separation; // speed_conversion_ değişkenine matris atıyoruz

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0; 
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = ros::Time::now();

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

void SimpleController::jointCallback(const sensor_msgs::JointState &state)
{
    double dp_left = state.position.at(0) - left_wheel_prev_pos_;
    double dp_right = state.position.at(1) - right_wheel_prev_pos_;
    
    double dt = (state.header.stamp - prev_time_).toSec();

    left_wheel_prev_pos_ = state.position.at(0);
    right_wheel_prev_pos_ = state.position.at(1);
    prev_time_ = state.header.stamp;

    double fi_left = dp_left / dt;  //rotational velocity (dönüş hızı olabilir araştır)
    double fi_right = dp_right / dt; 

    double linear_ = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
    double angular_ = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    double d_s = (wheel_radius_ * fi_right + wheel_radius_ * fi_left ) / 2;
    double d_theta = (wheel_radius_ * fi_right - wheel_radius_ * fi_left ) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear_;
    odom_msg_.twist.twist.angular.z = angular_;

    odom_pub_.publish(odom_msg_);

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = ros::Time::now();

    static tf2_ros::TransformBroadcaster br_;

    br_.sendTransform(transform_stamped_);



}
