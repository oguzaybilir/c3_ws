#include <ros/ros.h> // Gerekli kütüphaneyi include ediyoruz
#include "bumperbot_examples/AddTwoInts.h" // AddTwoInts.srv servis dosyamızı kodumuza include ediyoruz.

int main(int argc, char **argv) // Ana fonksiyonumuzu oluşturuyoruz.
{   // argc - argument count - argüman sayısı
    // argv - argument vector - argüman vektörü
    ros::init(argc, argv, "simple_service_client_cpp"); // ros::init sınıfını kullanarak koda ait node'u (düğümü) oluşturuyoruz.
    if(argc != 3) // eğer argc yani gelen argüman sayısı 3 değilse
    {
    // Aşağıda iki argüman lazım diyoruz lakin if koşulunda 3 argümandan bahsediyoruz. Bunun nedeni ilk argümanın her zaman script'in adı olmasıdır. 
    // Bu durumda terminalde 2 argüman verildiğinde argc değerinin 3 olarak döner.

        ROS_INFO("Requested two arguments"); // 2 argüman gerektiğine dair bilgi mesajının ekrana veriyoruz
        return 1; // Kodun hatalı sonuçlandığına dair return değeridir.
    }

    ros::NodeHandle nh; // Düğümlerimizle uğraşabilmek için ros::NodeHandle 'ı çağırıp nh değişkenine atıyoruz.
    ros::ServiceClient client = nh.serviceClient<bumperbot_examples::AddTwoInts>("add_two_ints"); // Servisler ile iş yapacağımız için bize bir client gerekiyor.
    // ros::ServiceClient sınıfına bağlı bir nh.serviceClient ögesine bumperbot_examples klasöründe bulunan AddTwoInts servis dosyasını veriyoruz.
    // Ardından bu servisi add_two_ints olarak isimlendiriyoruz.

    bumperbot_examples::AddTwoInts srv; // Servis dosyamızı srv değişkenine eşitliyoruz
    srv.request.a = atoi(argv[1]); // AddTwoInts.srv dosyasında bulunan request bölümüne ait bir a mesajımız var, atoi fonksiyonu ile bu değeri int yapıyoruz.
    // argv, yani argümanların 1. index'ini (yani ikinci elemanını) almamızın sebebi ilk argüman olarak script'in adının dönmesidir.
    srv.request.b = atoi(argv[2]); // AddTwoInts.srv dosyasında bulunan request bölümüne ait bir b mesajımız var, atoi fonksiyonu ile bu değeri int yapıyoruz.
    // argv, yani argümanların 2. index'ini (yani üçüncü elemanını) almamızın sebebi ilk argüman olarak script'in adının dönmesidir.

    ROS_INFO("Requesting %d + %d", srv.request.a, srv.request.b); // Terminale yazı yayınlanır. 
    if(client.call(srv)) // Eğer client servere başarı ile istek gönderirse
    {
        ROS_INFO("Service Response %d", srv.response.sum); // Servisin cevabı x tir şeklinde bir bilgi terminale yazdırılır.
    }

    else // Eğer client servere başarı ile istek gönderemezse
    {
        ROS_ERROR("Failed to call service add_two_ints"); // Terminale add_two_ints servisinin çağırılamadığına dair bir hata mesajı yayınlanır
        return 1; // Mantık olarak kod hatalı çalıştığı için yani istenen sonuç alınamadığı için return 1 olur
    }

    return 0;
}