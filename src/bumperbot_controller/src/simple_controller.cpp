// Gerekli kütüphaneleri ve header dosyalarını include ediyoruz
#include "bumperbot_controller/simple_controller.h" // simple_controller.h ismindeki header dosyamızı include ediyoruz, bu header dosyasında init ögeleri yer alıyor
#include <std_msgs/Float64.h> // Aracımızın sağ ve sol tekerine ait hız kontrolcülerinden yayınlanacak olan mesajlar Float64 tipindeki sayısal mesajlardır
#include <Eigen/Geometry> // Eigen (öz, lineer cebirdeki özdeğer, özvektör olayları) kütüphanesinden Geometry sınıfını include ediyoruz
#include <tf2/LinearMath/Quaternion.h> // tf2 kütüphanesindeki LinearMath sınıfından Quaternion fonksiyonunu çağırıyoruz, kullanım sebebi kalan kısımlarda açıklanmıştır
#include <tf2_ros/transform_broadcaster.h> // tf2_ros kütüphanesinden transform_broadcaster sınıfın çağırıyoruz, bu sınıfın kullanım sebebi kodun aşağı kısımlarında açıklanmıştır

SimpleController::SimpleController(const ros::NodeHandle &nh, double radius, double separation) :
            nh_(nh),
            wheel_radius_(radius),
            wheel_separation_(separation),
            left_wheel_prev_pos_(0.0),
            right_wheel_prev_pos_(0.0),
            x_(0.0),
            y_(0.0),
            theta_(0.0) // SimpleController sınıfının constructor fonksiyonunu oluşturuyoruz
            // Bu constructor sınıfına verilen parametreler simple_controller.h header dosyasından gelmektedir.
{
    ROS_INFO_STREAM("Using wheel radius " << radius); // ROS_INFO_STREAM fonksiyonu ile terminalde "Using wheel radius " yazdırıyoruz ve ardından radius değişkenini yazdırıyoruz
    ROS_INFO_STREAM("Using wheel separation " << separation); // ROS_INFO_STREAM fonksiyonu ile terminalde "Using wheel separation " yazdırıyoruz ve ardından separation değişkenini yazdırıyoruz


    right_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_controller/command", 10); // nh_ değişkeninden wheel_right_controller/command topic'ine 10 mesaj gönderilebileceğini belirtiyoruz
    left_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_controller/command", 10); // nh_ değişkeninden wheel_left_controller/command topic'ine 10 mesaj gönderilebileceğini belirtiyoruz

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("bumperbot_controller/odom", 10); // Odometry verilerini yayınlamak için odom_pub_ isminde bir değişken oluşturuyoruz
    vel_sub_ = nh_.subscribe("bumperbot_controller/cmd_vel", 1000, &SimpleController::velCallback, this); // nh_ değişkeninden bumperbot_controller/cmd_vel topic'inden 1000 mesaj alabileceğimizi belirtiyoruz ve velCallback fonksiyonunu çağırıyoruz

    joint_sub_ = nh_.subscribe("joint_states", 1000, &SimpleController::jointCallback, this); // Robotun jointlerinden gelen verileri dinlemek için joint_states topiğine abone oluyoruz

    speed_conversion_ << radius/2, radius/2, radius/separation, -radius/separation; // speed_conversion_ değişkenine matris atıyoruz

    // header dosyamızdan gelen odom_msg_ verilerine erişiyoruz
    // hangi frameden hangi frame'e yapılacak olan dönüşüm için linklerin isimlerini veriyoruz
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";

    // odom verilerimizden olan orientation (yön) verilerini oluşturuyoruz
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0; 
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0; // bu değerin 1 olmasının sebebi rotasyonun genel büyüklüğünü ve genel dönme açısını vermesidir

    // transform yapılacak olan frameleri belirliyoruz
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = ros::Time::now(); // zaman bağlı bir dönüşüm yapılacağı için zamanı güncellemek gerekir, bu nedenle şimdiki zamanı kullanıyoruz

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

void SimpleController::jointCallback(const sensor_msgs::JointState &state) // JointStates topiğinden gelen verileri alıp işlemek için gerekli callback fonksiyonunu yazıyoruz

{

    // dp_x, delta position of the x, dt ise delta time anlamına gelir.
    double dp_left = state.position.at(0) - left_wheel_prev_pos_; // sol tekerin anlık konumu - eski konumu hesaplaması ile sol teker konum farkı hesaplanır
    double dp_right = state.position.at(1) - right_wheel_prev_pos_; // sağ tekerin anlık konumu - eski konumu hesaplaması ile sağ teker konum farkı hesaplanır
    double dt = (state.header.stamp - prev_time_).toSec(); // anlık zaman - eski zaman hesaplamasından geçen zaman hesaplanır ve saniye cinsine çevrilir

    // hesaplamalar yapıldıktan sonra eski konum ve eski zaman bilgilerinin güncellenmesi gerekir ki bir sonraki iterasyonda yeni konum ve zaman bilgileri kullanılabilsin
    left_wheel_prev_pos_ = state.position.at(0); // sol tekerin anlık konumu
    right_wheel_prev_pos_ = state.position.at(1); // sağ tekerin anlık konumu
    prev_time_ = state.header.stamp; // anlık zaman bilgisi

    double fi_left = dp_left / dt;  //rotational velocity (dönüş hızı) sol tekere ait dönüş hızı
    double fi_right = dp_right / dt; // rotational velocity (dönüş hızı) sağ tekere ait dönüş hızı

    double linear_ = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2; // tüm robota ait lineer hız hesaplaması
    double angular_ = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_; // tüm robota ait açısal hız hesaplaması

    double d_s = (wheel_radius_ * fi_right + wheel_radius_ * fi_left ) / 2; // robotun konumundaki değişimi hesaplıyoruz
    double d_theta = (wheel_radius_ * fi_right - wheel_radius_ * fi_left ) / wheel_separation_; // robotun açısındaki değişimi hesaplıyoruz

    theta_ += d_theta; // anlık açı değerini açıdaki değişim ile toplayarak yeni anlık açıyı elde ediyoruz
    x_ += d_s * cos(theta_); // aracın x konumunu konumdaki değişiklik ile yeni anlık açının kosinüsünün çarpımı ile hesaplıyoruz
    y_ += d_s * sin(theta_); // aracın y konumunu konumdaki değişiklik ile yeni anlık açının sinüsünü çarpımı ile hesaplıyoruz

    // dönüşüm ve odometry hesaplanan kısıma geldik
    tf2::Quaternion q; // tf2 kütüphanesinden Quaternion sınıfın aldık ve q değişkenine atadık
    q.setRPY(0, 0, theta_); // quaterniondan RPY açılarına geçiyoruz
    odom_msg_.header.stamp = ros::Time::now(); // zamanı şimdiki zaman yapıyoruz

    // artık Quaternion açılarını kullandığımız için odom_msg_ objesindeki x, y, z ve w değerlerini quaternion matrisinden gelen verilere eşitliyoruz
    odom_msg_.pose.pose.orientation.x = q.getX(); 
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();

    // üst tarafta güncellenen x, y, lineer hız ve açısal hız değerlerini odom_msg_ verilerine veriyoruz
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear_;
    odom_msg_.twist.twist.angular.z = angular_;

    odom_pub_.publish(odom_msg_); // oluşturduğumuz odom_pub_ publisher ögesi ile odom_msg_ verilerini publishliyoruz

    // Yukarıda Quaternion oluşturduk ve Odometry verilerine verdiğimiz x, y, z, w değerlerinin aynısını transform_stamped_ ögesine ROTATION verisi olarak veriyoruz
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = ros::Time::now(); // zamanı şimdiki zaman olarak atıyoruz

    static tf2_ros::TransformBroadcaster br_; // tf2_ros kütüphanesine ait TransformBroadcaster sınıfını statik bir br_ değişkenine atadık
    // Statik değişken nedir ? 
    // Statik değişken yerel bir değişken tanımlama yöntemidir ancak normal yerel değişkenler hafıza tutulmazlar, her çağırıldıklarına baştan işleme sokulurlar.
    // Statik değişkenler yerel bir değişken olmalarına karşın hafızada tutulurlar ve bu sayede hep var oldukları değer ile kullanılmaya devame edebilirler.

    br_.sendTransform(transform_stamped_); // yaptığımız transformları yayınlıyoruz



}
