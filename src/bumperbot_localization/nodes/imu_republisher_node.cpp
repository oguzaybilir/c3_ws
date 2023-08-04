#include <ros/ros.h> // gerekli kütüphaneleri include ediyoruz
#include <sensor_msgs/Imu.h> 


ros::Publisher imu_pub_; // değişiklik yapılan imu verilerini yayınlayacak olan publisher'ı oluşturuyoruz

void imuCallback(const sensor_msgs::Imu &imu_) // imu topiğini dinleyen değişkenin gelen veriler ile ne yapacağını dair bir fonk yazıyoruz
{
    sensor_msgs::Imu new_imu_; // Imu tipinde bir new_imu_ değişkeni oluşturuyoruz
    new_imu_ = imu_; // new_imu_ değişkenini fonksiyona gelen verilere eşitliyoruz
    new_imu_.header.frame_id = "base_footprint_ekf"; // sensör verilerinin referans alınacağı frame'i veriyoruz
    imu_pub_.publish(new_imu_); // bu değişikliği publish ediyoruz
}

int main(int argc, char **argv) // main fonksiyonu yazıyoruz
{ 
    ros::init(argc, argv, "imu_republisher_node"); // init node oluşturup imu_republisher_node olarak isimlendiriyoruz
    ros::NodeHandle nh; // düğümlerle uğraşmak için ros NodeHandle oluşturuyoruz
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu_ekf", 10); // yapılan değişiklikleri yayınlıyoruz
    ros::Subscriber imu_sub_ = nh.subscribe("imu", 1000, imuCallback); // imu topiğine abone oluyoruz
    ros::spin(); // koda spin attırıyoruz
    return 0; // kod doğru çalıştı devam et
}