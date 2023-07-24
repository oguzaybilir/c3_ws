#include <ros/ros.h> // Gerekli kütüphaneleri include ediyoruz.

void timerCallback(const ros::TimerEvent &event) // Gerekli callback fonksiyonunu yazıyoruz.
{
    ROS_INFO_STREAM("Called timerCallback function "); // timerCallback fonksiyonu çağrıldı şeklinde bir bilgi mesajını terminale yazdırıyoruz
}

int main(int argc, char **argv) // Ana fonksiyonumuzu oluşturuyoruz
{
    ros::init(argc, argv, "simple_timer_cpp"); // init node'umuzu oluşturuyoruz ve simple_timer_cpp olarak isimlendiriyoruz.
    ros::NodeHandle nh; // Düğümlerimizle uğraşabilmek için NodeHandle'ı çağırıyoruz.
    ros::Duration timer_duration(1); // Duration tipinde bir timer_duration() fonk. oluşturup süreyi 1 sn olarak ayarlıyoruz.
    ros::Timer timer = nh.createTimer(timer_duration, timerCallback); // Zamanlayıcımızı oluşturup 1 sn aralıklarla çalıştırıyoruz
    // ve timerCallback ile 1 sn de bir bilgi mesajı yayınlıyoruz.
    ros::spin(); // Kodu sürekli olarak çalışmasını sağlıyoruz.
    return 0; // Kod başarı ile çalıştı.
}