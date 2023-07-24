// BU BİR HEADER DOSYASI OLDUĞU İÇİN PRİVATE ÖGELERE ERİŞİLEBİLİYOR, NASIL ERİŞİLEBİLİYOR ? 
// HEADER DOSYALARINA AİT PRİVATE FONKSİYONLARI .CPP DOSYALARI İÇİNDE ERİŞİLEBİLİYOR

#ifndef TF_EXAMPLES_H // if not defined (eğer tanımlı değilse) tf_examples.h dosyası
#define TF_EXAMPLES_H // tf_examples.h dosyasını tanımla

// gerekli kütüphaneleri include ediyoruz
#include <ros/ros.h> 
#include <geometry_msgs/TransformStamped.h> // zamana bağlı dinamik ve statik transformları yapıyoruz
// TransformStamped fonksiyonu ne işe yarar ? 
// Bu fonksiyon frame ile child frame arasındaki dönüşümü yapar

#include "bumperbot_examples/GetTransform.h" // oluşturduğumuz servis dosyasını buraya dahil ediyoruz

#include <tf2_ros/transform_listener.h> // tf2_ros kütüphanesine ait transform_listener (transform_dinleyici) fonksiyonunu dahil ediyoruz
// Bu fonksiyon dönüşümleri dinler

#include <tf2/LinearMath/Quaternion.h> // tf2 kütüphanesine ait Quaternion açısı sınıfını dahil ediyoruz

class TfExamples // TfExamples adında bir sınıf oluşturuyoruz
{


public: // sınıfa ait üyelik fonksiyonlardan olan public fonksiyonunu çağırıyoruz
// public : dışarıdan ve içeriden erişilebilen ögelerdir, bu sınıfı başka bir yere
    TfExamples(const ros::NodeHandle &);

private: // sınıfa ait üyelik fonksiyonlarından olan private fonksiyonunu çağırıyoruz
// private : sadece sınıf içinde kullanılan ögelerdir, sınıf dışında başka bir yerde görülemez, kullanılamaz
    ros::NodeHandle nh_; // ros node handle 'ı nh_ değişkenine eşitliyoruz


    geometry_msgs::TransformStamped static_transform_stamped_; // geometry_msgs sınıfından TransformStamped fonksiyonunu çağırıyoruz ve static_transform_stamped_ değişkenine eşitliyoruz

    // Static Transform nedir ? 
    // Birbirine fixed olarak bağlı, yani aynı anda aynı yolu kateden, aynı hıza sahip olan linkler arasındaki dönüşümdür. Örneğin base link ile kamera arasındaki dönüşüm statik bir dönüşümdür.
    // Çünkü bu linkler birbirleri ile aynı anda aynı yolu kateder, aynı hızlara sahiplerdir. Yani birbirlerine fiziksel olarak bağlıdırlar.

    geometry_msgs::TransformStamped dynamic_transform_stamped_; // geometry_msgs sınıfından TransformStamped fonksiyonunu çağırıyoruz ve dynamic_transform_stamped_ değişkenine eşitliyoruz
    // Dynamic Transform nedir ? 
    // Birbirine fixed olarak bağlı olmayan, yani aynı anda aynı yolu katetmeyen, aynı hıza sahip olmayan linkler arasındaki dönüşümdür. Örneğin referans noktası olarak bir odom linkimiz bulunsun.
    // Bu odom linki sabit iken base_link hareket halinde ise yapılması gereken dönüşüm Dynamic Transformdur.

    ros::Timer timer_; // Yapılacak olan dönüşüm için bize zaman lazım, çünkü world frame'inde zamana bağlı olarak her şey değişmektedir.

    double x_increment_; // Bizim sabit olarak belirlediğimiz değişkendir. Bu değişken x ekseninde alınacak yolu tutar. Her adımda ne kadar yol alınacağına dair bir değişkendir. 

    double last_x_; // Son x konumunu tutan değişkeni tanımladık.

    ros::ServiceServer get_transform_srv_; // ROS service server sınıfını çağırdık ve get_transform_srv_ değişkenine atadık.
    // ServiceServer ne işe yarar ? 
    // ServiceServer gerçek manada bizim service client bağlantısını kurduğumuz serverdır.

    tf2_ros::Buffer tf_buffer_; // tf2_ros kütüphanesinden Buffer sınıfını çağırdık ve tf_buffer_ değişkenine atadık.
    // Buffer sınıfı ne işe yarar ? 
    // Buffer sınıfı yapılan dönüşümleri bir süreliğine hafızada tutar.

    tf2_ros::TransformListener tf_listener_; // tf2_ros kütüphanesinden TransformListener sınıfını çağırdık ve tf_listener_ değişkenine atadık.
    // TransformListener sınıfı ne işe yarar ? 
    // Bu sınıf yapılan dönüşümleri dinler ve içinde hangi frame'den hangi frame'e dönüşüm yapıldığını dinler ve frameler arasındaki koordinat dönüşümlerini yapar.

    int rotation_counter_; // rotasyon yani dönüş sayısı değişkenini oluşturuyoruz.

    tf2::Quaternion last_orientation_; // robotun en son kaç derece olduğunun bilgisine dair bir değişken oluşturuyoruz.

    tf2::Quaternion orientation_increment_; // her adımda kaç dereecelik dönüş yapılacağına dair bir değişken oluşturuyoruz.

    
    void timerCallback(const ros::TimerEvent &); // ros::Timer() oluştururken bize bu zaman bilgisi ile ne yapılacağına dair bir callback fonksiyonu gerekir, bu nedenle bu fonksiyonu void olarak oluşturduk.
    // içini tf_examples.cpp dosyasında dolduracağız. 
    // içine const tipinde bir ros::TimerEvent sınıfı alır.
    // ros::TimerEvent sınıfı ne işe yarar ? 
    // TimerEvent sınıfı zamana bağlı bir olay için bize zamanı verir.

    bool getTransformCallback(bumperbot_examples::GetTransform::Request &,
                            bumperbot_examples::GetTransform::Response &); // nh_.advertiseService() fonksiyonun kullanırken bize bir callback fonksiyonu gerekir, bu nedenle bu fonksiyonu oluşturuyoruz.
    // bu fonksiyon bool tipinde çünkü GetTransform.srv / .h dosyamızda bool tipinde bir success response'u (cevabı var).
    // Bize dönüşümün başarılı olup olmadığına dair bilgi vermesi için bool tipinde success response'u ve bool tipinde bir callback fonksiyonu oluşturuyoruz.

};
#endif