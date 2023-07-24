
#include "turtlesim_kinematic.h" // turtlesim_kinematic.h dosyasını çağırıyoruz

int main(int argc, char **argv) // ana fonksiyonumuz
{
    ros::init(argc, argv, "turtlesim_kinematics_node"); // init node tanımlıyoruz
    TurtlesimKinematics turtlesim_kinematics; // TurtlesimKinematics sınıfından turtlesim_kinematics adında bir nesne oluşturuyoruz
    ros::spin(); // ros spin fonksiyonunu çağırıyoruz
    return 0; // ana fonksiyonumuzun dönüş değeri 0
}