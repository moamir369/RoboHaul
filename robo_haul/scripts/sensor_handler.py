#!/usr/bin/env python3

import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class SensorHandler : 
    def __init__(self):

        self.on_mesafe = 10.0
        self.sol_mesafe =10.0
        self.sag_mesafe =10.0

        self.x =0.0
        self.y = 0.0
        self.yaw = 0.0


    def scan_callback(self , msg: LaserScan):
        
        mesafeler = msg.ranges
        n = len(mesafeler)

        def clean(v):
            if math.isnan(v) or math.isinf(v) or v<0.05:
                return 10.0
            return v
        
        # (360 / 12 = 30) -> زاوية المنطقة الأمامية الكلية بالدرجات
        on_bolge_boyutu = n//12

        # (360 / 6 = 60) -> زاوية بداية منطقة اليسار بالدرجات
        sol_baslangic_derecesi = n//6

        # (360 / 3 = 120) -> زاوية نهاية منطقة اليسار بالدرجات
        sol_bitis_derecesi = n//3

        # (2 * 360 / 3 = 240) -> زاوية بداية منطقة اليمين بالدرجات
        sag_baslangic_derecesi = 2 * n // 3

        # (5 * 360 / 6 = 300) -> زاوية نهاية منطقة اليمين بالدرجات
        sag_bitis_derecesi = 5 * n // 6

        on_indeksler = (list(range(on_bolge_boyutu)) + list(range(n-on_bolge_boyutu , n)))
        sol_indeksler = range(sol_baslangic_derecesi , sol_bitis_derecesi)
        sag_indeksler = range(sag_baslangic_derecesi , sag_bitis_derecesi)

        on_gecerli_mesafeler = []
        sol_gecerli_mesafeler = []
        sag_gecerli_mesafeler = []

        for i in on_indeksler :
            temiz_deger = clean(mesafeler[i])
            on_gecerli_mesafeler.append(temiz_deger)
        self.on_mesafe = min(on_gecerli_mesafeler)

        for i in sol_indeksler :
            temiz_deger = clean(mesafeler[i])
            sol_gecerli_mesafeler.append(temiz_deger)
        self.sol_mesafe = min(sol_gecerli_mesafeler)

        for i in sag_indeksler :
            temiz_deger = clean(mesafeler[i])
            sag_gecerli_mesafeler.append(temiz_deger)
        self.sag_mesafe = min(sag_gecerli_mesafeler)


    def odom_callback(self, msg: Odometry):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

