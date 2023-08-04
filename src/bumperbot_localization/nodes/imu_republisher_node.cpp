#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


ros::Publisher imu_pub_;

void imuCallback(const sensor_msgs::Imu &imu_)
{
    sensor_msgs::Imu new_imu_;
    new_imu_ = imu_;
    new_imu_.header.frame_id = "base_footprint_ekf";
    imu_pub_.publish(new_imu_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_republisher_node");
    ros::NodeHandle nh;
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu_ekf", 10);
    ros::Subscriber imu_sub_ = nh.subscribe("imu", 1000, imuCallback);
    ros::spin();
    return 0;
}