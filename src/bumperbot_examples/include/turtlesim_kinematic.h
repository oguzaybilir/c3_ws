// İki tane turtlesim robotu arasındaki uzaklığı hesaplamak için yazdığımız sınıf 

// .h yani header dosyaları fonksiyonların, sınıfların, değişkenlerin, yapıların ve diğer program ögelerinin bildirimlerini içeren dosyalardır
// .h dosyaları genellikle .cpp dosyaları ile birlikte kullanılır. .cpp dosyaları .h dosyalarındaki bildirimleri kullanarak uygulamayı oluşturur.
// .h dosyaları genellikle sınıf bildirimlerini içerir. Sınıf bildirimleri, sınıfın üyelerini ve sınıfın nasıl kullanılacağını tanımlar.


#ifndef TURTLESIM_KINEMATICS_H_ // Eğer TURTLESIM_KINEMATICS_H_ tanımlanmamışsa aşağıdaki kodları çalıştır
#define TURTLESIM_KINEMATICS_H_ // TURTLESIM_KINEMATICS_H_ tanımla
#include <ros/ros.h> // ros kütüphanesini çağırıyoruz
#include <turtlesim/Pose.h> // turtlesim kütüphanesini çağırıyoruz

class TurtlesimKinematics // TurtlesimKinematics adında bir sınıf oluşturuyoruz
{

public: // public kısmı sınıf dışındanda erişilebilen kısımdır
    TurtlesimKinematics(); // TurtlesimKinematics sınıfının ana, yapıcı fonksiyonunu tanımlıyoruz

    void turtle1PoseCallback(const turtlesim::Pose& pose); // abone olunan turtle1/pose topicinden gelen verileri işleyeceğimiz callback fonksiyonunu tanımlıyoruz


    void turtle2PoseCallback(const turtlesim::Pose& pose); // abone olunan turtle2/pose topicinden gelen verileri işleyeceğimiz callback fonksiyonunu tanımlıyoruz

private: // private kısmı sadece sınıf içinden erişilebilen kısımdır
    ros::NodeHandle nh_; // ros node handle oluşturuyoruz, nodehandle ros ile iletişim kurmamızı sağlar
    ros::Subscriber turtle1_pose_sub_; // turtle1/pose topiğine abone oluyoruz
    ros::Subscriber turtle2_pose_sub_; // turtle2/pose topiğine abone oluyoruz

    turtlesim::Pose last_turtle1_pose_; // turtle1/pose topiğinden gelen verileri tutmak için bir turtlesim::Pose tipinde değişken tanımlıyoruz
    turtlesim::Pose last_turtle2_pose_; // turtle2/pose topiğinden gelen verileri tutmak için bir turtlesim::Pose tipinde değişken tanımlıyoruz


};



#endif // TURTLESIM_KINEMATICS_H_ // Eğer TURTLESIM_KINEMATICS_H_ tanımlanmışsa aşağıdaki kodları çalıştır