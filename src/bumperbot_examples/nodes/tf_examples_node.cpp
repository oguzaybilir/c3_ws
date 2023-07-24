#include "tf_examples.h" // Gerekli header dosyamızı include ettik.
#include <ros/ros.h> // Gerekli kütüphanemizi include ettik.


int main(int argc, char **argv) // Ana fonksiyonumuzu oluşturduk.
{
    ros::init(argc, argv, "tf_examples"); // init node umuzu oluşturuyoruz.
    ros::NodeHandle nh; // Düğümlerimizle uğraşmak için NodeHandle'ımızı çağırıyoruz.
    TfExamples tfExamples(nh); // TfExamples sınıfını çağırıp tfExamples() fonksiyonunu oluşturuyoruz
    // bu fonksiyona düğümlerimizi veriyoruz
    ros::spin(); // Kod sürekli çalışsın diye spin fonk kullanıyoruz.
    return 0; // Kod hatasız 
}