#include <ros/ros.h> // kütüphaneleri import ediyoruz
#include "bumperbot_controller/simple_controller.h" // SimpleController sınıfımızın header dosyasını import ediyoruz


int main(int argc, char **argv) // main fonksiyonumuzu oluşturuyoruz
{
    ros::init(argc, argv, "simple_controller"); // ros node'u başlatıyoruz
    ros::NodeHandle nh; // ros node handle oluşturuyoruz ve nh değişkenine atıyoruz
    ros::NodeHandle pnh("~"); // private değerlere erişmek için ros node handle oluşturuyoruz pnh değişkenine atıyoruz, bu pnh özel bir tanımlama
    double wheel_radius; // wheel_radius değişkenini tanımlıyoruz
    pnh.getParam("wheel_radius", wheel_radius); // pnh değişkeninden wheel_radius değerini alıyoruz
    double wheel_separation; // wheel_separation değişkenini tanımlıyoruz
    pnh.getParam("wheel_separation", wheel_separation); // pnh değişkeninden wheel_separation değerini alıyoruz
    SimpleController controller(nh, wheel_radius, wheel_separation); // SimpleController sınıfından controller değişkeni oluşturuyoruz ve parametreleri atıyoruz
    ros::spin(); // ros node'unu döndürüyoruz
    return 0; // 0 döndürüyoruz
}