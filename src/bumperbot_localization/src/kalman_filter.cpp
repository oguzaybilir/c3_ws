#include <ros/ros.h> // gerekli kütüphaneleri import ediyoruz
#include "bumperbot_localization/kalman_filter.h" // kalman_filter.h header dosyamızı include ediyoruz


KalmanFilter::KalmanFilter(const ros::NodeHandle &nh) // ana klası ve constructor fonksiyonu oluşturuyoruz
        :
        // header dosyasında tanımlanan değişkenlere init değer ataması yapıyoruz
        nh_(nh),
        mean_(0.0),
        variance_(1000.0),
        imu_angular_z_(0.0),
        is_first_odom_(true),
        last_angular_z_(0.0),
        motion_(0.0),
        motion_variance_(4.0),
        measurement_variance_(0.5)

{
    odom_sub_ = nh_.subscribe("bumperbot_controller/odom_noisy", 1000, &KalmanFilter::odomCallback, this); // odom_noisy topiğini dinleyecek olan subscriber'ı oluşturuyoruz
    imu_sub_ = nh_.subscribe("imu", 10, &KalmanFilter::imuCallback, this); // imu topiğine dinleyecek olan subscriber'ı oluşturuyoruz

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("bumperbot_controller/odom_kalman", 10); // kalman filtresiyle düzeltilmiş odom_kalman topiğini yayınlıyacak olan publisherı oluşturuyoruz
}

void KalmanFilter::imuCallback(const sensor_msgs::Imu& imu) // imu_sub_ subscriber'ının dinlediği sensör verileri ile işlem yapacağı fonksiyonu yazıyoruz
{
    imu_angular_z_ = imu.angular_velocity.z; // imu_angular_z_ değişkenine imu sensöründen gelen açısal hızın z elemanını (dönüş değeri) veriyoruz

}

void KalmanFilter::odomCallback(const nav_msgs::Odometry& odom) // odom_sub_ subscriber'ının dinlediği sensör verileri ile işlem yapacağı fonksiyonu yazıyoruz
{
    kalman_odom_ = odom; // fonksiyona gelen odom verisini kalman_odom_ değişkenine atadık
    if(is_first_odom_) // eğer is_first_odom_ değişkeni true ise
    {
        mean_ = odom.twist.twist.angular.z; // mean_ değerini odom angular_z değerine eşitledik
        last_angular_z_ = odom.twist.twist.angular.z; // değişkeni güncelliyoruz
        is_first_odom_ = false; // is_first odomu false yapıyoruz
        return;
    }

    motion_ = odom.twist.twist.angular.z - last_angular_z_; // motion_ değerini güncelliyoruz
    
    statePrediction(); // statePrediction() fonksiyonunu çalıştırıyoruz
    measurementUpdate(); // measurementUpdate() fonksiyonunu çalıştırıyoruz

    kalman_odom_.twist.twist.angular.z = mean_; // kalman_odom_ mesajının z elemanını mean_ e eşitliyoruz
    odom_pub_.publish(kalman_odom_); // kalman_odom_ değişkenini publish ediyoruz

    last_angular_z_ = odom.twist.twist.angular.z; // last_angular_z_ değişkenini odom mesajının z elemanına eşitliyoruz

}

// Kalman filtresinin birinci aşaması olan fonksiyonu yazıyoruz
void KalmanFilter::measurementUpdate() // measurementUpdate() fonksiyonunu yazıyoruz
{
    mean_ = (measurement_variance_ * mean_ + variance_ * imu_angular_z_) / (variance_ + measurement_variance_); // measurementUpdate() için mean_ değerini güncelliyoruz
    variance_ = (variance_ * measurement_variance_) / (variance_ + measurement_variance_); // measurementUpdate() için variance_ değerini güncelliyoruz
}

// Kalman filtresinin ikinci aşaması olan fonksiyonu yazıyoruz
void KalmanFilter::statePrediction() // statePrediction() fonksiyonunu yazıyoruz
{
    mean_ = mean_ + variance_; // statePrediction() için mean_ değerini güncelliyoruz
    variance_ = variance_ + motion_variance_; // statePrediction() için variance_ değerini güncelliyoruz
}