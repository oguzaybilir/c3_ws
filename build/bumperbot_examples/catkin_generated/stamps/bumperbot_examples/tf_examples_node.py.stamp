#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy # gerekli kütüphaneyi import ediyoruz
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster #tf2_ros kütüphanesinden StaticTransformBroadcaster ve TransformBroadcaster sınıflarını import ediyoruz
# Bunlar transformları yani dönüşümleri yayınlayacak olan sınıflarımızdır.
from geometry_msgs.msg import TransformStamped # Bu sınıf zamana bağlı dönüşüm yapan sınıftır.
from bumperbot_examples.srv import GetTransform, GetTransformResponse # Bizim oluşturduğumuz .srv servis dosyasını import ediyoruz

from tf2_ros import TransformListener, Buffer # tf2_ros kütüphanesinden TransformListener ve Buffer sınıflarını import ediyoruz
# TransformListener sınıfı yapılan transformları dinler ve belirli bir süre hafızada tutar
# Buffer, yapılan transformların kaçlı olarak ve ne kadar süre hafızada tutulacağını belirlediğimiz sınıftır.

# Bu fonksiyonlar euler açılarından quaternion açılarına geçiş yapılırken kullanılan fonksiyonlardır.
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
# quaternion_from_euler : euler açılarından quaternion açılarına geçişi yapar
# quaternion_multiply : self.last_orientation_ ile self.orientation_increment_ değişlenlerini çarpmak için kullandığımız fonksiyondur
# bu değerlerin çarpılmasının sebebi ise quaternion matrisini elde etmektir.
# quaternion_inverse : quaternion matrisini terse çevirerek yapılan hareketi terse çevirir. (alfa açısı kadar sağa dön, alfa açısına gelince inverse al ve alfa açısı kadar sola dön)


class TfExamples(object): # sınıfımızı oluşturduk

    def __init__(self): # init fonksiyonumuzu oluşturduk
        self.static_broadcaster_ = StaticTransformBroadcaster() # StaticTrasnformBroadcaster() sınıfını self.static_broadcaster_ değişkenine atadık
        # Static Transform nedir ? 
        # Birbirine fixed olarak bağlı olan linkler arasındaki dönüşümdür. Yani aynı anda aynı yöne ve aynı hıza sahip linkler arasında yapılması gereken dönüşümdür.
        # self.static_broadcaster_ değişkenini dönüşüm yapmak için kullanmıyoruz, bu değişkenin görevi dönüşüm yapmak değil, yayınlamaktır.

        self.dynamic_broadcaster_ = TransformBroadcaster() # TransformBroadcaster() sınıfını self.dynamic_broadcaster_ değişkenine atadık.
        # Dynamic Transform nedir ? 
        # Birbirine fixed olarak bağlı olmayan, aynı anda aynı yöne ve aynı hızda hareketlenmeyen linkler arasında yapılması gereken dönüşümdür.
        # self.dynamic_broadcaster_ değişkenini dönüşüm yapmak için kullanmıyoruz, bu değişkenin görevi dönüşüm yapmak değil, yayınlamaktır.

        self.static_transform_stamped_ = TransformStamped() # self.static_transform_stamped_ değişkenine TransformStamped() sınıfını atadık
        self.dynamic_transform_stamped_ = TransformStamped() # self.dynamic_transform_stamped_ değişkenine TransformStamped() sınıfını atadık

        self.timer_ = rospy.Timer(rospy.Duration(0.1), self.timerCallback) # timer oluşturduk çünkü Transformlarımız zamana bağlı olarak yapılıyor.

        self.x_increment_ = 0.05 # self.x_increment_ değişkenine x ekseninde her adımda gidilecek yolu atadık.
        self.last_x_ = 0.0 # self.last_x_ değişkenine başlangıç değeri olarak 0.0 atadık, bu değişken bizim son x konumumuzu tutuyor.

        self.rotation_counter_ = 0.0 # self.rotation_counter_ değişkenine başlangıc değeri olarak 0.0 atadık, bu değişken bizim dönüş açımızı sayıyor.
        self.last_orientation_ = quaternion_from_euler(0, 0, 0) # self.last_orientation yani son yön bilgimizi quaternion_from_euler fonksiyonunun çıktısına eşitledik.
        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05) # self.orientation_increment_ değişkenini quaternion_from_euler fonksiyonunun çıktısına eşitledik.
        # bunun sebebi her adımda hangi euler eksenininde (bizim normal kullandığımız eksen) kaç derecelik açı ile dönüş yapacağımızı quaternion eksenine dönüştürmektir.


        #-----------------STATIC TRANSFORM----------------#
        # Static Transform'un timerCallback içinde olmamasının sebebi zamana bağlı bir değişimin olmamasıdır.
        
        self.static_transform_stamped_.header.stamp = rospy.Time.now() # stamp header'ını şuanki zamana eşitledik.
        self.static_transform_stamped_.header.frame_id = "bumperbot_base" # frame_id mizi bumperbot_base'e eşitledik.
        self.static_transform_stamped_.child_frame_id = "bumperbot_top" # child_frame_id mizi bumperbot_top'a eşitledik.

        # Buradaki tanımlamalar eksen etrafındaki dönüşler değildir. Linklerimizin konumlarıdır.
        # TRANSLATION = KONUM BILGISI
        self.static_transform_stamped_.transform.translation.x = 0.0 # linklerimizin (frame_id ve child_frame_id) x eksenindeki dönüş değerini verdik.
        self.static_transform_stamped_.transform.translation.y = 0.0 # linklerimizin (frame_id ve child_frame_id) y eksenindeki dönüş değerini verdik.
        self.static_transform_stamped_.transform.translation.z = 0.3 # linklerimizin (frame_id ve child_frame_id) z eksenindeki dönüş değerini verdik.

        # Buradaki tanımlamalar eksenler etrafındaki dönüşlerdir. 
        # ROTATION = DONUS BILGISI 
        self.static_transform_stamped_.transform.rotation.x = 0.0 # linklerimizin (frame_id ve child_frame_id) x eksenindeki dönüş değerini verdik.
        self.static_transform_stamped_.transform.rotation.y = 0.0 # linklerimizin (frame_id ve child_frame_id) y eksenindeki dönüş değerini verdik.
        self.static_transform_stamped_.transform.rotation.z = 0.0 # linklerimizin (frame_id ve child_frame_id) z eksenindeki dönüş değerini verdik.
        self.static_transform_stamped_.transform.rotation.w = 1.0 # linklerimizin (frame_id ve child_frame_id) w eksenindeki dönüş değerini verdik.

        self.static_broadcaster_.sendTransform(self.static_transform_stamped_) # static transformlarımıza ait dönüşümleri yayınladık.

        rospy.loginfo("Publishing static transform between %s and %s",
                    self.static_transform_stamped_.header.frame_id,
                    self.static_transform_stamped_.child_frame_id) # hangi frameler arasında dönüşüm yapıldığına dair bir bilgi mesajını ekrana yazdırdık.
        
        self.get_transform_srv_ = rospy.Service("get_transform",GetTransform, self.getTransformCallback) # Yapılan dönüşümleri servisimiz ile yayınladık.
        self.tf_buffer_ = Buffer() # Sürekli dolup boşalan ve belirli bir süre içinde veri tutan bir buffer sınıfına self.tf_buffer_ değişkenini atadık.
        self.tf_listener_ = TransformListener(self.tf_buffer_) # Transformlarımızı dinleyen ve bunlar ile dönüşüm işlemini yapan sınıfımıza self.tf_buffer_
        # değişkenini parametre olarak verdik.


    #------------------------DYNAMIC TRANSFORM-----------------------#
    # Dynamic Transformların timerCallback içinde olması zamana bağlı bir değişim olmasıdır.
        
    def timerCallback(self, event): # servisimizin aldığı verileri kullanacağı callback fonksiyonunu oluşturuyoruz.
        self.dynamic_transform_stamped_.header.stamp = rospy.Time.now() # zaman header'ına şimdiki zamanı verdik.
        self.dynamic_transform_stamped_.header.frame_id = "odom" # frame id mizi verdik.
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base" # child_frame_id mizi verdik.

        #euler
        # NEDEN Euler ekseni kullanıyoruz ? 
        # Euler eksenini robotun x y z eksenindeki ilerleyişi için kullanırız.
        # Dönüşümlerimiz dinamik olduğu için hareket yapılan eksendeki konumların güncellenmesi gerekiyor.
        # Bu nedenle euler açılarımızı güncellememiz gerekiyor.
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_ # aracın x eksenindeki konumunu güncelliyoruz.
        self.dynamic_transform_stamped_.transform.translation.y = 0.0 # y ekseninde bir hareket olmadığı için statik olarak 0 kalması gerekmektedir.
        self.dynamic_transform_stamped_.transform.translation.z = 0.0 # z ekseninde bir hareket olmadığı için statik olarak 0 kalması gerekmektedir.

        #quarternion
        # BURADAKİ DEĞERLER DENEME AMAÇLI DEĞİŞKEN OLMAYAN DEĞERLERDİR. AŞAĞIDA DEĞİŞKEN OLAN HALLERİ VARDIR VE ONLARIN KULLANILMASI GEREKMEKTEDİR.
        # self.dynamic_transform_stamped_.transform.rotation.x = 0
        # self.dynamic_transform_stamped_.transform.rotation.y = 0
        # self.dynamic_transform_stamped_.transform.rotation.z = 0
        # self.dynamic_transform_stamped_.transform.rotation.w = 1

        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)
        # q, yani quaternion matrisini oluşturmak için son yön bilgisi ile hangi yönde ne kadarlık adımlarda ilerleneceğine dair matrislerin çarpılması gerekmektedir.
        self.dynamic_transform_stamped_.transform.rotation.x = q[0] # quaternion matrisinin ilk elemanı bize x eksenindeki dönüş bilgisini verir.
        self.dynamic_transform_stamped_.transform.rotation.y = q[1] # quaternion matrisinin ikinci elemanı bize y eksenindeki dönüş bilgisini verir.
        self.dynamic_transform_stamped_.transform.rotation.z = q[2] # quaternion matrisinin üçüncü elemanı bize z eksenindeki dönüş bilgisini verir.
        self.dynamic_transform_stamped_.transform.rotation.w = q[3] # quaternion matrisinin dördüncü elemanı bize aracın genel dönüş bilgisini verir.

        self.dynamic_broadcaster_.sendTransform(self.dynamic_transform_stamped_) # Yaptığımız dynamic transformları yayınladık.
        
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x # son x konumumuzu self.last_x_ değişkenine atadık, böylece son x konumunu güncellemiş olduk.
        self.rotation_counter_ += 1 # self.rotation_counter_ değişkenini bir arttırdık, bu sayede dönüş sayacını doğru bir şekilde ayarlamış olduk.
        self.last_orientation_ = q # self.last_orientation_ değişkenini Quaternion matrisine eşitledik, çünkü q bize aracın Rotation Matrix'ini veriyor.

        if self.rotation_counter_ >= 100: # eğer dönüş sayacı 100 e eşit veya büyük olursa
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_) # dönüş miktarını terse çevir (+alfa dereceden -alfa derece dönüştür.)
            self.rotation_counter_ = 0 # sayacı sıfırla

    def getTransformCallback(self, req): # Dönüşümlerimizin başarılı olup olmadığına dair bir callback fonksiyonu oluşturuyoruz.
        rospy.loginfo("Requested transform between %s and %s ", req.frame_id, req.child_frame_id) # sisteme hangi frameden hangi frame'e dönüşüm yapıldığına dair bir bilgi geçiyoruz.
        res = GetTransformResponse() # GetTransform.srv dosyamızın cevabı olan res'i GetTransforResponse()'a eşitledik.
        requested_transform_ = TransformStamped() # istenen dönüşümü TransformStamped() sınıfına eşitledik.

        # Herhangi bir hata alınması durumunda sistemin kapanmaması için bir try except bloğu yazıyoruz.
        try:
            requested_transform_ = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rospy.Time()) # zamana bağlı bir dönüşüm yapıldığı için son parametre olarka rospy.Time() verildi.
            # tf_buffer_ sınıfına ait lookup_transform fonksiyonunu kullanarak dönüşümü tamamlıyoruz.        
        except Exception as e: # hata gelmesini bekle
            rospy.logerr("An error occured while trasnforming %s and %s", req.frame_id, req.child_frame_id) # sisteme hangi frame'den hangi frame'e dönüşüm yapılırken hata alındığına dair bir hata bilgisi yazdırıyoruz.
            res.success = False # hata alındığı için GetTransform.srv dosyamızda bulunan res isimli cevaplardan success bilgisini false olarak tanımlıyoruz.
            return res # res cevabını döndürüyoruz.
        
        rospy.loginfo("The requested tranform is : %s", requested_transform_) # istenen dönüşümü bilgi mesajı olarak sisteme veriyoruz.
        res.transform = requested_transform_ # GetTransform.srv dosyamızın transform bilgisini requested_transform_ değişkenine atıyoruz.
        res.success = True # GetTransform.srv dosyamızda bulunan success cevabını True olarak atadık çünkü dönüşüm başarılı oldu.

        return res # res cevabını döndür



if __name__ == "__main__": 
    rospy.init_node("tf_examples") # init node oluşturup tf_examples olarak isimlendirdik.
    tfExamples = TfExamples() # tfExamples ögesini TfExamples() sınıfına eşitledik.
    rospy.spin() # spin atarak kodun sürekli çalışmasını sağladık.

