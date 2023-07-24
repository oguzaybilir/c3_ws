#include "tf_examples.h" // oluşturduğumuz header dosyasını include ediyoruz.
#include <ros/ros.h> // gerekli kütüphaneleri include ediyoruz.
#include <geometry_msgs/TransformStamped.h> // TransformStamped 
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

TfExamples::TfExamples(const ros::NodeHandle &nh) : nh_(nh), x_increment_(0.05), last_x_(0.0), tf_listener_(tf_buffer_), rotation_counter_()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    static_transform_stamped_.header.stamp = ros::Time::now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";

    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;

    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    static_broadcaster.sendTransform(static_transform_stamped_);
    ROS_INFO_STREAM("Publishing static transform between " << static_transform_stamped_.header.frame_id  <<
                    " and "  <<  static_transform_stamped_.child_frame_id);


    timer_ = nh_.createTimer(ros::Duration(0.1), &TfExamples::timerCallback, this);

    get_transform_srv_ = nh_.advertiseService("get_transform", &TfExamples::getTransformCallback, this);

    last_orientation_.setRPY(0, 0, 0);
    orientation_increment_.setRPY(0, 0, 0.05);

}


void TfExamples::timerCallback(const ros::TimerEvent &)
{
    static tf2_ros::TransformBroadcaster dynamic_broadcaster;

    dynamic_transform_stamped_.header.stamp = ros::Time::now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";

    dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;

    // dynamic_transform_stamped_.transform.rotation.x = 0.0;
    // dynamic_transform_stamped_.transform.rotation.y = 0.0;
    // dynamic_transform_stamped_.transform.rotation.z = 0.0;
    // dynamic_transform_stamped_.transform.rotation.w = 1.0;

    tf2::Quaternion q;
    q = last_orientation_ * orientation_increment_;
    q.normalize();
    
    dynamic_transform_stamped_.transform.rotation.x = q.x();
    dynamic_transform_stamped_.transform.rotation.y = q.y();
    dynamic_transform_stamped_.transform.rotation.z = q.z();
    dynamic_transform_stamped_.transform.rotation.w = q.w();


    dynamic_broadcaster.sendTransform(dynamic_transform_stamped_);
    last_x_ = dynamic_transform_stamped_.transform.translation.x;

    rotation_counter_ ++;
    last_orientation_ = q;


    if(rotation_counter_ >= 100)
    {
        orientation_increment_ = orientation_increment_.inverse();
        rotation_counter_ = 0;
    }


}


bool TfExamples::getTransformCallback(bumperbot_examples::GetTransform::Request &req,
                        bumperbot_examples::GetTransform::Response &res)
{
    ROS_INFO_STREAM("Requested transform between " << req.frame_id << " and " << req.child_frame_id);
    geometry_msgs::TransformStamped requested_transform_;

    try
    {
        requested_transform_ = tf_buffer_.lookupTransform(req.frame_id, req.child_frame_id, ros::Time());
    }


    catch(const tf2::TransformException &ex)
    {
        ROS_ERROR_STREAM("An error occured while transforming " << req.frame_id << " and " << req.child_frame_id << " : " << ex.what());
        res.success = false;
        return true;
    }

    ROS_INFO_STREAM("The requested transform is \n" << requested_transform_);
    res.transform = requested_transform_;
    res.success = true;
    return true;

}