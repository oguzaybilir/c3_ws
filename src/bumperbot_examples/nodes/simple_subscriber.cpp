// kütüphanleri import ediyoruz
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>


void msgCallback(const std_msgs::String::ConstPtr& msg) // abone olunan topicten gelen verileri işleyeceğimiz callback fonksiyonunu tanımlıyoruz
{
    ROS_INFO("New message received: %s", msg->data.c_str()); // gelen veriyi ekrana yazdırıyoruz
}

int main(int argc, char **argv) // ana fonksiyonumuz
{

    ros::init(argc, argv, "simple_subscriber_cpp"); // init node tanımlıyoruz
    ros::NodeHandle n; // ros node handle çağırıp n harfine eşitliyoruz
    ros::Subscriber sub = n.subscribe("chatter",10, msgCallback); // chatter topiğine abone oluyoruz ve gelen verileri msgCallback fonksiyonuna gönderiyoruz
    ros::spin(); // ros spin fonksiyonunu çağırıyoruz

    return 0; // ana fonksiyonumuzun dönüş değeri 0
}