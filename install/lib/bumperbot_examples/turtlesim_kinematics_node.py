#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# kütüphaneleri import ediyoruz
import rospy
from turtlesim.msg import Pose
from math import sin, cos , pi, radians, atan2

class TurtlesimKinematics(object): # TurtlesimKinematics adında bir sınıf oluşturuyoruz

    def __init__(self): # sınıfa ait init fonksiyonunu oluşturuyoruz
        self.turtle1_pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.turtle1PoseCallback) # turtle1/pose topicine abone oluyoruz
        self.turtle2_pose_sub = rospy.Subscriber("/turtle2/pose", Pose, self.turtle2PoseCallback) # turtle2/pose topicine abone oluyoruz

        # turtle1/pose topiğinden gelen veriler Pose tipinde olduğu için o verilere erişmek adına
        # self.last_turtle1_pose_ değişkenini Pose()'a eşitliyoruz
        self.last_turtle1_pose_  = Pose()
        self.last_turtle2_pose_  = Pose() 

        # callback fonksiyonlarımızı çağırıyoruz ve içlerine gerekli argümanları veriyoruz
        self.turtle1PoseCallback(self.last_turtle1_pose_)
        self.turtle2PoseCallback(self.last_turtle2_pose_)


    # turtle1/pose topicinden gelen verileri Pose tipindeki pose değişkenine atıyoruz
    def turtle1PoseCallback(self, pose):
        self.last_turtle1_pose_ = pose


    # turtle2/pose topicinden gelen verileri Pose tipindeki pose değişkenine atıyoruz
    def turtle2PoseCallback(self, pose):
        self.last_turtle2_pose_ = pose

        tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x # turtle1 ve turtle2 arasındaki x uzaklığını hesaplıyoruz
        ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y # turtle1 ve turtle2 arasındaki y uzaklığını hesaplıyoruz

        th_rads = atan2(ty, tx) # turtle1 ve turtle2 arasındaki açıyı hesaplıyoruz

        th_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta # turtle1 ve turtle2 arasındaki açıyı hesaplıyoruz
        th_rads = radians(th_rad) # açıyı radyan cinsinden hesaplıyoruz
        th_deg = th_rads * 180.0 / pi #     açıyı derece cinsinden hesaplıyoruz

        rospy.loginfo("""--------------------------------------------\n 
                    Translation Vector turtle1 -> turtle2\n
                    tx: %f\n
                    ty: %f\n
                    Rotation Matrix turtle1 -> turtle2\n
                    theta (rad): %f\n
                    theta (deg): %f\n
                    [ R11      R12] : [%f        %f]\n]
                    [ R21      R22] : [%f        %f]\n]""",
                    tx, ty, th_rad, th_deg,
                    cos(th_rad), -sin(th_rad),
                    sin(th_rad), cos(th_rad))

if __name__ == "__main__": # eğer bu dosya main dosya ise
    rospy.init_node("turtlesim_kinematics_node", anonymous=False) # init_node tanımlaması yapıyoruz ve isimlendiriyoruz
    turtlesim_kinematics = TurtlesimKinematics() # TurtlesimKinematics sınıfından turtlesim_kinematics adında bir nesne oluşturuyoruz
    rospy.spin() # rospy kapanana kadar döngüyü çalıştırıyoruz



