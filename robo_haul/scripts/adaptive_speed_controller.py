#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from sensor_handler import SensorHandler

#  Mesafe eşikleri (metre) 
DANGER_DIST  = 0.35   # Bu değerin altında → anında dur
SLOW_DIST    = 0.70   # Bu değerin altında → yavaş hız
MEDIUM_DIST  = 1.20   # Bu değerin altında → orta hız
CLEAR_DIST   = 2.00   # Bu değerin üstünde → maksimum hız

#  Hız oranları (0.0 ile 1.0 arasında) 
SPEED_STOP   = 0.0    # Tehlike
SPEED_SLOW   = 0.25   # Yavaş
SPEED_MEDIUM = 0.60   # Orta
SPEED_FULL   = 1.00   # Tam hız

#  Hız yumuşatma 
ALPHA = 0.3   # Küçüldükçe geçiş daha yavaş ve yumuşak olur (0.1–0.5)


class AdaptiveSpeedController(Node):

    def __init__(self):
        super().__init__('adaptive_speed_controller')

        #  Sensör işleyici 
        self.sensors = SensorHandler()

        self.create_subscription(LaserScan, '/scan',        self.sensors.scan_callback,  10)
        self.create_subscription(Twist,     '/cmd_vel_nav', self.nav_cmd_callback, 10)

        #  Yayıncı → Robot 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #  İç durum değişkenleri 
        self.current_speed_ratio = 1.0
        self.target_speed_ratio  = 1.0
        self.last_nav_cmd        = Twist()

        #  Zamanlayıcı: düzenli aralıklarla komut gönder (20 Hz) 
        self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info("Uyarlanabilir Hız Kontrolcüsü başlatıldı")
        self.get_logger().info(
            f"   Tehlike={DANGER_DIST}m  Yavaş={SLOW_DIST}m  "
            f"Orta={MEDIUM_DIST}m  Açık={CLEAR_DIST}m"
        )

    def nav_cmd_callback(self, msg: Twist):
        self.last_nav_cmd = msg

    def _distance_to_ratio(self, dist: float) -> float:
        if dist <= DANGER_DIST:
            return SPEED_STOP

        elif dist <= SLOW_DIST:
            t = (dist - DANGER_DIST) / (SLOW_DIST - DANGER_DIST)
            return SPEED_STOP + t * (SPEED_SLOW - SPEED_STOP)

        elif dist <= MEDIUM_DIST:
            t = (dist - SLOW_DIST) / (MEDIUM_DIST - SLOW_DIST)
            return SPEED_SLOW + t * (SPEED_MEDIUM - SPEED_SLOW)

        elif dist <= CLEAR_DIST:
            t = (dist - MEDIUM_DIST) / (CLEAR_DIST - MEDIUM_DIST)
            return SPEED_MEDIUM + t * (SPEED_FULL - SPEED_MEDIUM)

        else:
            return SPEED_FULL

    def _get_zone(self) -> str:
        d = self.sensors.on_mesafe
        if   d <= DANGER_DIST:  return "TEHLİKE"
        elif d <= SLOW_DIST:    return "YAVAŞ  "
        elif d <= MEDIUM_DIST:  return "ORTA   "
        else:                   return "AÇIK   "

    def publish_cmd(self):
        self.target_speed_ratio = self._distance_to_ratio(self.sensors.on_mesafe)

        # Yumuşatma: hedefe doğru kademeli geçiş
        self.current_speed_ratio += ALPHA * (
            self.target_speed_ratio - self.current_speed_ratio
        )

        modified           = Twist()
        modified.angular.z = self.last_nav_cmd.angular.z
        original_linear    = self.last_nav_cmd.linear.x

        if self.current_speed_ratio <= 0.01:
            modified.linear.x = 0.0
            if abs(original_linear) > 0.01:
                self.get_logger().warn(
                    f" TEHLİKE! Ön mesafe={self.sensors.on_mesafe:.2f}m → DURDU",
                    throttle_duration_sec=1.0
                )
        else:
            modified.linear.x = original_linear * self.current_speed_ratio
            self.get_logger().info(
                f"[{self._get_zone()}] mesafe={self.sensors.on_mesafe:.2f}m "
                f"oran={self.current_speed_ratio:.2f} "
                f"hız={modified.linear.x:.3f}m/s",
                throttle_duration_sec=0.5
            )

        self.cmd_pub.publish(modified)


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