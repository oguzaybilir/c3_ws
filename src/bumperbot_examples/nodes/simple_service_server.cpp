#include <ros/ros.h> // Gerekli kütüphaneyi include ediyoruz.
#include "bumperbot_examples/AddTwoInts.h" // bumperbot_examples dizininde bulunan AddTwoInts.srv dosyasını AddTwoInts.h olarak include ediyoruz.


bool addCallback(bumperbot_examples::AddTwoInts::Request &req,
                bumperbot_examples::AddTwoInts::Response &res) // Server elemanımıza ait callback fonksiyonunu tanımlıyoruz.
{
    ROS_INFO("Ready to sum %d and %d", (int)req.a, (int)req.b); // x ve y 'yi toplamaya hazır biçiminde bir bilgiyi terminale veriyoruz
    res.sum = req.a + req.b; // sum, a ve b değerleri AddTwoInts.srv dosyamızda bulunuyorlar, değişken değiller, statikler.
    return true; // işlem tamamlandığı için true döndürüyoruz
}

int main(int argc, char **argv) // Ana fonksiyonumuzu yazıyoruz
{
    ros::init(argc, argv, "simple_service_server_cpp"); // Server düğümünü oluşturuyoruz.
    ros::NodeHandle nh; // Düğümlerimizle uğraşmak için NodeHandle'ı çağırıyoruz ve nh değişkenine atıyoruz.
    ros::ServiceServer service = nh.advertiseService("add_two_ints", addCallback); // ros::ServiceServer sınıfı tipinde bir servis değişkeni oluşturuyoruz ve bunu
    // nh.advertiseService() fonksiyonuna eşitliyoruz.
    // Burada yapılan işlem add_two_ints adında bir service elemanını server-client ilişkisinde server yapmaktır.
    ROS_INFO("The service is ready to add two ints "); // bilgi mesajı
    ros::spin(); // İşlemin sürekli olarak yapılması için kullandığımız fonksiyon.
    return 0; // Kod başarı ile çalıştı.
}