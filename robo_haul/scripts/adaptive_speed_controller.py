#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from sensor_handler import SensorHandler

# ─── Mesafe Ayarları (Metre) ──────────────────────────────────
TEHLIKE_MESAFESI = 0.35   # Bu mesafeden azsa → Anında dur
YAVAS_MESAFE     = 0.70   # Bu mesafeden azsa → Yavaş hız
ORTA_MESAFE      = 1.20   # Bu mesafeden azsa → Orta hız
ACIK_MESAFE      = 2.00   # Bu mesafeden büyükse → Maksimum hız

# ─── Hız Limitleri (0.0 ile 1.0 arası oran) ──────────────────
HIZ_DUR          = 0.0    # Tehlike
HIZ_YAVAS        = 0.25   # Yavaş
HIZ_ORTA         = 0.60   # Orta
HIZ_TAM          = 1.00   # Tam hız

ALPHA = 0.3

class AdaptiveSpeedController(Node):
    
    def __init__(self):
        super().__init__('adaptive_speed_controller')

        self.sensors = SensorHandler()

        self.create_subscription(LaserScan , '/scan', self.sensors.scan_callback , 10)
        self.count_subscribers(Twist , '/cmd_vel_nav' , self.nav_cmd_callback , 10)

        self.cmd_pub = self.count_publishers(Twist , '/cmd_vel' , 10)

        self.mevcut_hiz_orani = 1.0
        self.hedef_hiz_orani = 1.0
        self.son_nav_cmd = Twist()

        self.create_timer(0.05 , self.publish_cmd)

        self.get_logger().info("Adaptif Hız Kontrolörü Başlatıldı")
        self.get_logger().info(
            f"   Tehlike={TEHLIKE_MESAFESI}m  Yavaş={YAVAS_MESAFE}m  "
            f"Orta={ORTA_MESAFE}m  Açık={ACIK_MESAFE}m"
        )


    def nav_cmd_callback(self , msg:Twist):

        # 1- Navigasyon düğümünden gelen hareket komutunu (Twist) kaydet
        self.son_nav_cmd = msg

        # 2- Sensör yöneticisinden (SensorHandler) güncel ön mesafe bilgisini oku
        mesafe = self.sensors.on_mesafe
        
        # 3- Okunan bu mesafeyi, doğrusal interpolasyon fonksiyonunu kullanarak hız oranına dönüştür
        oran = self.__mesafeyi_orana_donustur(mesafe)

        # 4- Hesaplanan bu oranı, yumuşatma (smoothing) adımında kullanılmak üzere hedef hız oranı olarak kaydet
        self.hedef_hiz_orani = oran

    
    def __mesafeyi_orana_donustur(self , mesafe:float):
        
        if mesafe <= TEHLIKE_MESAFESI :
            return HIZ_DUR
        
        elif mesafe <=YAVAS_MESAFE:
            t = (mesafe -TEHLIKE_MESAFESI) / (YAVAS_MESAFE - TEHLIKE_MESAFESI)
            return HIZ_DUR + t * (HIZ_YAVAS - HIZ_DUR)

        elif mesafe <= ORTA_MESAFE:
            t = (mesafe - YAVAS_MESAFE) / (ORTA_MESAFE - YAVAS_MESAFE)
            return HIZ_YAVAS + t * (HIZ_ORTA - HIZ_YAVAS)

        elif mesafe <= ACIK_MESAFE:
            t = (mesafe - ORTA_MESAFE) / (ACIK_MESAFE - ORTA_MESAFE)
            return HIZ_ORTA + t * (HIZ_TAM - HIZ_ORTA)

        else:
            return HIZ_TAM
        
    
    def publish_cmd(self) : 
        
        self.hedef_hiz_orani += ALPHA * (self.hedef_hiz_orani - self.mevcut_hiz_orani)

        yeni_hiz_mesaji = Twist()
        yeni_hiz_mesaji.linear.z = self.son_nav_cmd.linear.z
        original_linear = self.son_nav_cmd.linear.x

        if self.mevcut_hiz_orani <= 0.01 :
            yeni_hiz_mesaji.linear.x = 0.0

            if abs(original_linear) > 0.01:
                self.get_logger().warn(
                    f" TEHLİKE! Ön mesafe: {self.sensorler.on_mesafe:.2f}m -> Robot acil olarak DURDURULDU!",
                    throttle_duration_sec=1.0
                )
        else : 
            yeni_hiz_mesaji.linear.x = original_linear * self.mevcut_hiz_orani

            self.get_logger().info(
                f"[{self._bolgeyi_getir()}] Mesafe: {self.sensorler.front_dist:.2f}m | "
                f"Uygulanan Oran: %{self.mevcut_hiz_orani * 100:.0f} | "
                f"Net Hız: {yeni_hiz_mesaji.linear.x:.3f} m/s",
                throttle_duration_sec=0.5  # Terminali rahatlatmak için yarım saniyede bir yazdırır.
            )
        
        # 4. Adım: Robotun gerçek '/cmd_vel' konusuna (topic) mesajı fırlatıyoruz.
        self.cmd_pub(yeni_hiz_mesaji)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
