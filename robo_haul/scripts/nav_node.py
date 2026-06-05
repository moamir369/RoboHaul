#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from astar_planner  import AStarPlanner
from sensor_handler import SensorHandler


class NavNode(Node):

    def __init__(self):
        super().__init__('simple_nav')

        #  Harita dosya yolu 
        MAP_YAML = '/home/emir/robo_haul_ws/src/robo_haul/maps/factory_map.yaml'

        #  Yol planlayıcı 
        self.planner = AStarPlanner(MAP_YAML)

        #  Sensör işleyici 
        self.sensors = SensorHandler()

        #  Yayıncılar (Publishers) 
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel_nav',   10)
        self.path_pub = self.create_publisher(Path,  '/global_path',   10)

        self.create_subscription(LaserScan, '/scan',       self.sensors.scan_callback, 10)
        self.create_subscription(Odometry,  '/odom',       self.sensors.odom_callback, 10)
        self.create_subscription(Point,     '/goal_point', self.goal_cb,               10)

        #  Navigasyon durumu 
        self.path       = []
        self.current_wp = 0
        self.active     = False

        #  Navigasyon parametreleri 
        self.SAFE_DIST   = 0.55   # Engellere güvenli mesafe (metre)
        self.LINEAR_VEL  = 0.45   # İleri hareket hızı (m/s)
        self.ANGULAR_VEL = 1.0    # Dönüş hızı (rad/s)
        self.WP_TOL      = 0.35   # Ara nokta tamamlanma toleransı (metre)

        #  Kontrol döngüsü (10 Hz) 
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('NAVİGASYON HAZIR')

    def goal_cb(self, msg: Point):
        """Yeni hedef al ve yolu planla"""

        gx, gy = msg.x, msg.y
        self.get_logger().info(f'Hedefe planlanıyor: {gx:.2f}, {gy:.2f}')

        path, actual_goal = self.planner.planla(
            self.sensors.x,
            self.sensors.y,
            gx, gy
        )

        #  Yol bulunamadıysa (Hedef tamamen kapalıysa) 
        if path is None:
            self.get_logger().warn(
                'YOL BULUNAMADI — Bölge çok dar ve yakınlarda alternatif bir güvenli nokta yok!'
            )
            return

        #  Alternatif hedefe yönlendirildiyse kullanıcıyı bilgilendir 
        agx, agy = actual_goal
        if abs(agx - gx) > 0.05 or abs(agy - gy) > 0.05:
            self.get_logger().warn(
                f'Hedef bölgesi dar ← Orijinal konum ({gx:.2f}, {gy:.2f}) yerine '
                f'en yakın güvenli noktaya ({agx:.2f}, {agy:.2f}) yönlendiriliyor.'
            )

        self.path       = path
        self.current_wp = 0
        self.active     = True

        #  Yolu RViz için yayınla 
        path_msg = Path()
        path_msg.header.frame_id = 'odom'  
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in path:
            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        self.get_logger().info(f'YOL HAZIR — {len(path)} ara nokta')

    def _normalize(self, a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def control_loop(self):
        if not self.active or not self.path:
            return

        wp_x, wp_y = self.path[self.current_wp]

        dx   = wp_x - self.sensors.x
        dy   = wp_y - self.sensors.y
        dist = math.sqrt(dx * dx + dy * dy)

        angle_to_wp = math.atan2(dy, dx)
        angle_error = self._normalize(angle_to_wp - self.sensors.yaw)

        cmd = Twist()

        #  Ara noktaya ulaşıldı 
        if dist < self.WP_TOL:
            self.current_wp += 1

            if self.current_wp >= len(self.path):
                self.active = False
                self.cmd_pub.publish(Twist())
                self.get_logger().info('HEDEFE ULAŞILDI')

            return

        if self.sensors.on_mesafe < self.SAFE_DIST:
            cmd.linear.x  = 0.0
            cmd.angular.z = (
                self.ANGULAR_VEL if self.sensors.sol_mesafe > self.sensors.sag_mesafe
                else -self.ANGULAR_VEL
            )

        #  Büyük açı düzeltmesi gerekiyor 
        elif abs(angle_error) > 0.25:
            cmd.linear.x  = 0.05
            cmd.angular.z = math.copysign(self.ANGULAR_VEL, angle_error)

        #  Normal seyir, duvar düzeltmesiyle 
        else:
            duvar_duzeltme = 0.0
            if self.sensors.sol_mesafe  < 0.60: duvar_duzeltme -= 0.15
            if self.sensors.sag_mesafe < 0.60: duvar_duzeltme += 0.15

            cmd.linear.x  = self.LINEAR_VEL
            cmd.angular.z = angle_error * 0.8 + duvar_duzeltme

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