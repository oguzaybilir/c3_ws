#include <ros/ros.h> // gerekli kütüphaneşeri include ediyoruz
#include "bumperbot_localization/kalman_filter.h" // header dosyamızı include ediyoruz

int main(int argc, char **argv) // main fonksiyonu yazıyoruz
{
    ros::init(argc, argv, "kalman_filter_node"); // init_node oluşturuyoruz
    ros::NodeHandle nh; // düğümlerimizle uğraşabilmek için ros NodeHandle'ı çağırıyoruz
    KalmanFilter filter(nh); // KalmanFilter sınıfını filter constructoruna atayıp düğümlerimizi parametre olarak veriyoruz
    ros::spin(); // kodun sürekli dönmesi için spin attırıyoruz
    return 0;
}