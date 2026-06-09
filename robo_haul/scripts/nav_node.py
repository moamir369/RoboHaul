#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import PoseStamped

from astar_planner import AStarPlanner
from sensor_handler import SensorHandler


class NavNode(Node):

    def __init__(self):
        super().__init__('nav_node')

        HARITA_YAML = '/home/emir/robo_haul_ws/src/robo_haul/maps/factory_map.yaml'

        self.planner = AStarPlanner(HARITA_YAML)

        self.sensors = SensorHandler()

        #Publishers
        self.cmd_pub = self.create_publisher(Twist , 'cmd_vel_nav' ,10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        #Subscribers 
        self.create_subscription(LaserScan,'/scan', self.sensors.scan_callback, 10)
        self.create_subscription(Odometry,'/odom', self.sensors.odom_callback, 10)
        self.create_subscription(Point, '/goal_point', self.yeni_hedef_rota, 10)

        #  Navigasyon Durum Değişkenleri 
        self.path = []
        self.mevcut_hedef_nokta = 0
        self.active = False

        #  Navigasyon Parametreleri (Ayarlar) 
        self.GUVENLI_MESAFE  = 0.55   # Engellerden uzak durma mesafesi (metre)
        self.ILERI_HIZ       = 0.45   # Düz gitme hızı (m/s)
        self.DONUS_HIZI      = 1.0    # Dönüş hızı (rad/s)
        self.HEDEF_TOLERANS  = 0.35   # Noktaya ulaşıldı sayılması için gereken mesafe (metre)

        self.create_timer(0.1, self.control_loop)


    def yeni_hedef_rota(self , msg:Point):
        
        hedef_x ,hedef_y = msg.x,msg.y
        self.get_logger().info(f'Yeni Hedefe Yol Planlanıyor: {hedef_x:.2f}, {hedef_y:.2f}')

        path , gercek_hedef =self.planner.plan(
            self.sensors.x,
            self.sensors.y,
            hedef_x , hedef_y
        )

        if path is None :
            self.get_logger().warn('YOL BULUNAMADI: Hedef noktaya ulaşılamıyor!')
            return
        
        self.path = path
        self.mevcut_hedef_nokta =0
        self.active =True

        #  الجزء الجديد لنشر المسار لـ RViz 
        path_msg = Path()
        path_msg.header.frame_id = 'odom'  # الـ frame الأساسي لخريطتك
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        gercek_hx, gercek_hy = gercek_hedef
        if abs(gercek_hx - hedef_x) > 0.05 or abs(gercek_hy - hedef_y) > 0.05:
            self.get_logger().warn(
                f'Hedef bölgesi dar! En yakın güvenli nokta olan '
                f'({gercek_hx:.2f}, {gercek_hy:.2f}) koordinatına yönlendiriliyor.'
            )

        self.get_logger().info(f' Rota başarıyla oluşturuldu. {len(path)} adet nokta belirlendi.')



    def _normalize(self, aci: float) -> float:
        
        while aci> math.pi : 
            aci -= 2 *math.pi

        while aci> math.pi :
            aci+= 2 * math.pi
        
        return aci
    


    def control_loop(self):
        if not self.active or not self.path:
            return
        
        hedef_x, hedef_y = self.path[self.mevcut_hedef_nokta]

        dx = hedef_x - self.sensors.x
        dy = hedef_y - self.sensors.y
        mesafe = math.sqrt(dx**2 + dy**2)

        hedefe_aci = math.atan2(dy, dx)
        aci_hatasi = self._normalize(hedefe_aci - self.sensors.yaw)

        cmd = Twist()

        if mesafe < self.HEDEF_TOLERANS:
            self.mevcut_hedef_nokta += 1

        # Rotadaki tüm noktalar bittiyse dur
        if self.mevcut_hedef_nokta >= len(self.path):
            self.active = False
            self.cmd_pub.publish(Twist()) # Robotu durdur
            self.get_logger().info('Hedefe başarıyla ulaşıldı!')
            return
        

        # B) Acil durum: Önümüzde engel varsa kaçın
        if self.sensors.on_mesafe < self.GUVENLI_MESAFE:
            cmd.linear.x = 0.0 
            # Hangi taraf daha boşsa o tarafa dön
            cmd.angular.z = self.DONUS_HIZI if self.sensors.sol_mesafe > self.sensors.sag_mesafe else -self.DONUS_HIZI
            self.get_logger().warn('Engel algılandı, kaçınma manevrası yapılıyor!')


        elif abs(aci_hatasi) > 0.25:
            cmd.linear.x  = 0.05
            cmd.angular.z = math.copysign(self.DONUS_HIZI, aci_hatasi)

        # D) Normal sürüş: Rota üzerinde ilerle ve duvarlara çarpmamak için hafif düzeltme yap
        else:
            duvar_duzeltme = 0.0
            if self.sensorler.left_dist  < 0.60: duvar_duzeltme -= 0.15 # Sola çok yakınsan sağa kır
            if self.sensorler.right_dist < 0.60: duvar_duzeltme += 0.15 # Sağa çok yakınsan sola kır

            cmd.linear.x = self.ILERI_HIZ
            cmd.angular.z = (aci_hatasi * 0.8) + duvar_duzeltme

        # 5. Adım: Hazırlanan komutu yayınla
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
