#ifndef KALMAN_FILTER_H // eğer kalman_filter.h dosyası tanımlanmadıysa
#define KALMAN_FILTER_H // kalman_filter.h dosyasını tanımla

// gerekli kütüphaneleri tanımla
#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class KalmanFilter // sınıfı oluştur
{
public: // public değişkenleri tanımla
    KalmanFilter(const ros::NodeHandle  &); // sınıfın constructor fonksiyonunu tanımlıyoruz

private: // private değişkenleri tanımla
    ros::NodeHandle nh_; // düğümlerle oynayabilmek için ros NodeHandle'ı çağırıp nh_ değişkenine atıyoruz
    ros::Subscriber odom_sub_; // nav_msgs paketindeki Odometry sınıfına abone olacak olan subscriberı oluşturuyoruz
    ros::Subscriber imu_sub_; // sensor_msgs paketindeki Imu sınıfına abone olacak olan subscriberı oluşturuyoruz
    ros::Publisher odom_pub_; // üzerinde değişiklik yapacağımız odom verilerini yayınlamak için publisher oluşturuyoruz

    double mean_; // ortalama değer değişkeni
    double variance_; // varyans (standart sapmanın karesi) değişkeni oluşturuyoruz
    double imu_angular_z_; // imu sensöründen gelen açısal hızı alıyoruz
    bool is_first_odom_; // işleme sokulacak olan odom verisini kontrol etmek için is_first_odom_ değişkenini oluşturuyoruz
    double last_angular_z_; // açısal hızı güncellemek ve son açısal hızı hafızada tutmak için bu değişkeni oluşturuyoruz
    double motion_; // hareket adında bir değişken oluşturuyoruz
    nav_msgs::Odometry kalman_odom_; // Odometry sınıfına ait bir değişken oluşturuyoruz

    // different video 1
    double motion_variance_; // hareket varyansı ile işlem yapabilmek için bir değişken oluşturuyoruz
    double measurement_variance_; // ölçüm varyansı ile değerleri güncellemek için bir değişken oluşturuyoruz


    void odomCallback(const nav_msgs::Odometry &); // odom topiğine abone olduktan sonra gelen verilerle işlem yapacak olan fonksiyonu yazıyoruz
    void imuCallback(const sensor_msgs::Imu &); // imu topiğine abone olduktan sonra gelen verilerle işle yapacak olan fonksiyonu yazıyoruz

    void measurementUpdate(); // kalman filtresinin birinci aşaması olan fonksiyonu yazıyoruz
    void statePrediction(); // kalman filtresinin ikinci aşaması olan fonksiyonu yazıyoruz
};
#endif