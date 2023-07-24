#include <ros/ros.h> // ros ile c++ ın haberleşmesini sağlamak için çağırdığımız kütüphane
#include <std_msgs/String.h> // std_msgs den aldığımız String türündeki mesajları import ediyoruz
#include <sstream> // string stream kütüphanesidir


int main(int argc, char **argv) // kodumuzun ana fonksiyonunu oluşturuyoruz
{
    ros::init(argc, argv, "simple_publisher_cpp");
    // ana düğümümüzü oluşturduk
    // birinci parametre ana fonksiyonun parametresi
    // ikinci parametrede ana fonksiyonun parametresi
    // üçüncü parametre düğümün adı

    ros::NodeHandle n;
    // bu sınıf bizim düğümlerimizi ros mastera tanıtmamızı sağlıyor

    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 10);
    // Publisher sınıfımızı oluşturduktan sonra n, NodeHandler'ın advertise fonksiyonu
    // ile mesajımızı çağırıyoruz, daha sonra Publisher düğümüne topiğimizin adını veriyoruz

    ros::Rate rate(10); 
    // Rate fonksiyonu sistemin çalışacağı hz değeridir, sistemin belirli periyodik
    // aralıklarla uyuması gerekir çünkü sürekli olarak veri yayınlamak hz ile ilgili
    // sorun çıkarabilir

    int counter = 0;
    // bu counter değişkeni bizim mesajlarımızı sayacak

    while(ros::ok()) // ros tamamken, yani ros çalışır durumdayken
    {
        std_msgs::String msg;
        // std_msg paketinden String sınıfını çağırdık ve bu sınıf msg değişkenine atadık

        std::stringstream ss;
        // stringStream yani sstream kütüphanesindeki stringStream sınıfını alıp ss değişkenine atadık

        ss << "Hello world from C++ " << counter;
        // ss sınıfının "Hello World from C++ {counter}" mesajını yayınlamasını sağladık

        msg.data = ss.str();
        // mesajımızın içindeki datayı ss değişkenini tuttuğu stringe eşitledik

        pub.publish(msg);
        // publisherımızın mesajımızı yayınlamasını sağladık

        ros::spinOnce();
        // sistemin sadece bir kere çalışmasını sağlıyoruz

        rate.sleep();
        // döngüyü uykuya alıyoruz
        
        counter++;
        // her döngüde counter değerini 1 arttırıyoruz

    }
    return 0;
}
