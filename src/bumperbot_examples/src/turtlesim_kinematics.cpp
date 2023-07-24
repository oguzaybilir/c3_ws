// turtlesim_kinematic.h header dosyasını import ediyoruz
#include "turtlesim_kinematic.h"


TurtlesimKinematics::TurtlesimKinematics() // TurtlesimKinematics sınıfının constructor fonksiyonunu oluşturuyoruz
{
    turtle1_pose_sub_ = nh_.subscribe("/turtle1/pose", 1000, &TurtlesimKinematics::turtle1PoseCallback, this); // turtle1/pose topiğine abone oluyoruz
    turtle2_pose_sub_ = nh_.subscribe("/turtle2/pose", 1000, &TurtlesimKinematics::turtle2PoseCallback, this); // turtle2/pose topiğine abone oluyoruz
    // sondaki this parametresi sınıfın kendisini işaret eden bir işaretçidir
    // this, sınıfın üye işlevlerinde kullanılabilir.
}

void TurtlesimKinematics::turtle1PoseCallback(const turtlesim::Pose& pose) // callback fonksiyonumuzu tanımlıyoruz
{
    last_turtle1_pose_ = pose; // turtle1/pose topiğinden gelen verileri last_turtle1_pose_ değişkenine atıyoruz
}

void TurtlesimKinematics::turtle2PoseCallback(const turtlesim::Pose& pose) //   callback fonksiyonumuzu tanımlıyoruz
{
    last_turtle2_pose_ = pose; // turtle2/pose topiğinden gelen verileri last_turtle2_pose_ değişkenine atıyoruz

    float tx = last_turtle2_pose_.x - last_turtle1_pose_.x; // turtle2/pose topiğinden gelen verileri last_turtle2_pose_ değişkenine atıyoruz
    float ty = last_turtle2_pose_.y - last_turtle1_pose_.y; //  turtle2/pose topiğinden gelen verileri last_turtle2_pose_ değişkenine atıyoruz

    float th_r = last_turtle2_pose_.theta - last_turtle1_pose_.theta; // theta açısının radyan cinsinden hesaplamak için iki robot arasındaki 
                                                                      // açı değerini birbirinden çıkarıyoruz
    float th_d = 180 * th_r / 3.14; // theta değerini derece cinsine dönüştürüyoruz

    ROS_INFO_STREAM("--------------------------------\n" <<
                    "Translation Vector from turtle1 --> turtle2: \n" <<
                    "Tx: " << tx << "\n" <<
                    "Ty: " << ty << "\n" <<
                    "Rotation Matrix turtle1 --> turtle2\n" <<
                    "th_r: " << th_r << "\n" <<
                    "th_d: " << th_d << "\n" <<
                    "[R11       R12]: [" << std::cos(th_r) << "\t" << -std::sin(th_r) << "]\n" <<
                    "[R21       R22]: [" << std::sin(th_r) << "\t" << std::cos(th_r) << "]\n");
}
