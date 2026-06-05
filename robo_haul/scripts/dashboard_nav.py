#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from flask import Flask, render_template_string, request, jsonify, send_file



# Harita Ayarları

MAP_IMAGE      = "/home/emir/robo_haul_ws/src/robo_haul/maps/factory_map.png"
MAP_RESOLUTION = 0.05       # metre/piksel
MAP_ORIGIN_X   = -6.9       # Harita başlangıç noktası X
MAP_ORIGIN_Y   = -5.91      # Harita başlangıç noktası Y
MAP_WIDTH      = 277        # Harita genişliği (piksel)
MAP_HEIGHT     = 236        # Harita yüksekliği (piksel)
DISPLAY_SCALE  = 2          # Görüntüleme ölçeği
DISPLAY_WIDTH  = MAP_WIDTH  * DISPLAY_SCALE
DISPLAY_HEIGHT = MAP_HEIGHT * DISPLAY_SCALE


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        
        # Hedefi simple_nav'a /goal_point üzerinden gönder
        self.goal_pub = self.create_publisher(Point, '/goal_point', 10)

        # /cmd_vel üzerinden doğrudan acil durdurma
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Abonelikler (Subscribers)
        
        # Odometri'den robot konumu
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Mesafeleri göstermek için LiDAR verisi
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        
        # Durum Değişkenleri
        
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_speed = 0.0
        self.status      = "Boşta"

        # LiDAR mesafeleri (ön, sol, sağ)
        self.front_dist = 99.0
        self.left_dist  = 99.0
        self.right_dist = 99.0

        # Görev istatistikleri
        self.mission_start    = None
        self.mission_elapsed  = 0.0
        self.mission_distance = 0.0
        self.missions_done    = 0
        self.prev_x = 0.0
        self.prev_y = 0.0

        # Olay günlüğü
        self.alerts = []

        # Kayıtlı hızlı hedefler
        self.saved_goals = [
            {"name": "Teslimat Bölgesi", "x": 5.5,  "y": 4.5},
            {"name": "Raf 1",            "x": -4.0, "y": 3.5},
            {"name": "Raf 2",            "x": -4.0, "y": 0.8},
            {"name": "Yükleme Bölgesi",  "x": -5.5, "y": -4.5},
        ]

        self.get_logger().info(" Dashboard düğümü hazır!")
    
    # Odometri İşleyici
    def odom_cb(self, msg):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # Aktif görev varsa kat edilen mesafeyi hesapla
        if self.mission_start is not None:
            dx = new_x - self.prev_x
            dy = new_y - self.prev_y
            d  = math.sqrt(dx*dx + dy*dy)
            if d < 2.0:  # Büyük sıçramaları yoksay
                self.mission_distance += d
            self.mission_elapsed = time.time() - self.mission_start

        self.prev_x    = new_x
        self.prev_y    = new_y
        self.robot_x   = new_x
        self.robot_y   = new_y

        # Twist'ten hızı hesapla
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.sqrt(vx*vx + vy*vy)

    
    # LiDAR İşleyici
    def scan_cb(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        def temizle(v):
            if math.isnan(v) or math.isinf(v) or v < 0.05:
                return 99.0
            return v

        fs = n // 12
        front_idx = list(range(fs)) + list(range(n - fs, n))
        left_idx  = range(n // 6, n // 3)
        right_idx = range(2 * n // 3, 5 * n // 6)

        self.front_dist = min(temizle(ranges[i]) for i in front_idx)
        self.left_dist  = min(temizle(ranges[i]) for i in left_idx)
        self.right_dist = min(temizle(ranges[i]) for i in right_idx)

    
    # Hedef Gönder
    def send_goal(self, x, y, name="Hedef"):
        msg   = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.goal_pub.publish(msg)

        # Görev takibini başlat
        self.mission_start    = time.time()
        self.mission_distance = 0.0
        self.prev_x = self.robot_x
        self.prev_y = self.robot_y
        self.status = "Hareket Ediyor"

        self.add_alert(f"Hedef → {name} ({x:.2f}, {y:.2f})", "info")
        self.get_logger().info(f" Hedef gönderildi: {name} x={x:.2f}, y={y:.2f}")

    
    # Robotu Durdur
    
    def stop_robot(self):
        # Boş Twist gönder = tam durdurma
        self.vel_pub.publish(Twist())
        self.vel_pub.publish(Twist())
        self.vel_pub.publish(Twist())

        self.status        = "Durduruldu"
        self.mission_start = None
        self.add_alert("Robot operatör tarafından durduruldu", "warning")
        self.get_logger().info(" Robot durduruldu")

    
    # Günlüğe Olay Ekle
    
    def add_alert(self, message, level="info"):
        self.alerts.append({
            "msg":   message,
            "level": level,
            "time":  time.strftime("%H:%M:%S")
        })
        if len(self.alerts) > 50:
            self.alerts.pop(0)

    
    # Görev İstatistikleri
    
    def get_stats(self):
        elapsed = 0.0
        if self.mission_start is not None:
            elapsed = time.time() - self.mission_start
        return {
            "elapsed":   round(elapsed, 1),
            "distance":  round(self.mission_distance, 2),
            "completed": self.missions_done,
        }



# ROS'u Ayrı Bir İş Parçacığında Çalıştır

rclpy.init()
node = DashboardNode()
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

app = Flask(__name__)


# HTML

HTML = r"""
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Robo Haul</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&family=Barlow+Condensed:wght@400;600;700;900&display=swap" rel="stylesheet">
<style>
:root {
    --bg:       #080c10;
    --s1:       #0c1118;
    --s2:       #111820;
    --border:   #1a2535;
    --border2:  #243040;
    --acc:      #00e5ff;
    --acc2:     #ff6b35;
    --green:    #00e676;
    --red:      #ff1744;
    --yellow:   #ffd740;
    --text:     #cdd9e5;
    --muted:    #3d5068;
    --mono:     'JetBrains Mono', monospace;
    --sans:     'Barlow Condensed', sans-serif;
}

*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    height: 100vh;
    overflow: hidden;
    display: flex;
    flex-direction: column;
}

/*  ÜST BAR  */
.topbar {
    height: 52px;
    background: var(--s1);
    border-bottom: 1px solid var(--border);
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0 24px;
    flex-shrink: 0;
}

.logo {
    font-family: var(--sans);
    font-size: 22px;
    font-weight: 900;
    letter-spacing: 3px;
    color: var(--acc);
    display: flex;
    align-items: center;
    gap: 10px;
}

.logo-dot {
    width: 8px; height: 8px;
    background: var(--acc);
    border-radius: 50%;
    box-shadow: 0 0 12px var(--acc);
    animation: blink 2s infinite;
}

@keyframes blink {
    0%,100%{ opacity:1; transform:scale(1); }
    50%{ opacity:0.4; transform:scale(1.4); }
}

.topbar-right {
    display: flex;
    align-items: center;
    gap: 20px;
    font-family: var(--mono);
    font-size: 12px;
}

.top-stat {
    display: flex;
    align-items: center;
    gap: 8px;
    color: var(--muted);
}
.top-stat .val {
    color: var(--acc);
    font-weight: 700;
    font-size: 14px;
}

.sys-tag {
    background: rgba(0,229,255,0.07);
    border: 1px solid rgba(0,229,255,0.2);
    color: var(--acc);
    font-size: 11px;
    padding: 4px 12px;
    border-radius: 3px;
    letter-spacing: 2px;
}

/*  DÜZEN  */
.layout {
    flex: 1;
    display: grid;
    grid-template-columns: 260px 1fr 280px;
    overflow: hidden;
}

/*  SOL KENAR ÇUBUĞU  */
.sidebar {
    background: var(--s1);
    border-right: 1px solid var(--border);
    overflow-y: auto;
    display: flex;
    flex-direction: column;
    gap: 0;
}

.sec {
    padding: 16px 18px;
    border-bottom: 1px solid var(--border);
}

.sec-label {
    font-family: var(--mono);
    font-size: 10px;
    letter-spacing: 2px;
    color: var(--muted);
    margin-bottom: 12px;
    text-transform: uppercase;
}

/* DURUM KARTI */
.state-grid {
    display: flex;
    flex-direction: column;
    gap: 2px;
}

.state-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 7px 10px;
    border-radius: 4px;
    background: var(--s2);
}

.state-key {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    letter-spacing: 1px;
}

.state-val {
    font-family: var(--mono);
    font-size: 13px;
    font-weight: 700;
    color: var(--text);
}

.state-val.acc  { color: var(--acc); }
.state-val.grn  { color: var(--green); }
.state-val.red  { color: var(--red); }
.state-val.ylw  { color: var(--yellow); }

/* LiDAR ÇUBUĞU */
.lidar-bars {
    display: flex;
    flex-direction: column;
    gap: 6px;
    margin-top: 4px;
}

.bar-row {
    display: flex;
    align-items: center;
    gap: 8px;
}

.bar-lbl {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    width: 40px;
    flex-shrink: 0;
}

.bar-track {
    flex: 1;
    height: 5px;
    background: var(--border);
    border-radius: 3px;
    overflow: hidden;
}

.bar-fill {
    height: 100%;
    border-radius: 3px;
    transition: width 0.3s;
    background: var(--green);
}

.bar-fill.warn   { background: var(--yellow); }
.bar-fill.danger { background: var(--red); }

.bar-val {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text);
    width: 36px;
    text-align: right;
    flex-shrink: 0;
}

/* İSTATİSTİKLER */
.stats-4 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
}

.sbox {
    background: var(--s2);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 10px 10px 8px;
    text-align: center;
}

.sbox .n {
    font-family: var(--mono);
    font-size: 20px;
    font-weight: 700;
    color: var(--acc);
    line-height: 1;
}

.sbox .u {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--muted);
    margin-top: 3px;
    letter-spacing: 1px;
}

/* HIZLI HEDEFLER */
.qgoal {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 9px 12px;
    border: 1px solid var(--border);
    border-radius: 6px;
    margin-bottom: 6px;
    cursor: pointer;
    transition: all 0.15s;
    background: var(--s2);
    font-family: var(--sans);
    font-size: 14px;
    font-weight: 600;
    color: var(--text);
    letter-spacing: 0.5px;
}

.qgoal:hover {
    border-color: var(--acc);
    background: rgba(0,229,255,0.05);
    transform: translateX(4px);
    color: var(--acc);
}

.qgoal .qdot {
    width: 6px; height: 6px;
    background: var(--acc2);
    border-radius: 50%;
    flex-shrink: 0;
}

.qgoal .qcoords {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    margin-left: auto;
}

/* HEDEF EKLE */
.add-form { display:flex; flex-direction:column; gap:7px; }
.form-row { display:flex; gap:7px; }

.inp {
    flex: 1;
    background: var(--s2);
    border: 1px solid var(--border);
    border-radius: 5px;
    padding: 7px 10px;
    color: var(--text);
    font-family: var(--mono);
    font-size: 11px;
    outline: none;
    transition: border-color 0.2s;
}
.inp:focus { border-color: var(--acc); }
.inp::placeholder { color: var(--muted); }

.btn-add {
    padding: 9px;
    background: rgba(0,229,255,0.07);
    border: 1px solid rgba(0,229,255,0.2);
    border-radius: 5px;
    color: var(--acc);
    font-family: var(--sans);
    font-size: 13px;
    font-weight: 700;
    letter-spacing: 1px;
    cursor: pointer;
    transition: all 0.15s;
}
.btn-add:hover { background: rgba(0,229,255,0.14); }

/*  HARİTA  */
.main {
    display: flex;
    flex-direction: column;
    padding: 16px;
    overflow: hidden;
}

.map-card {
    flex: 1;
    background: var(--s1);
    border: 1px solid var(--border);
    border-radius: 12px;
    overflow: hidden;
    display: flex;
    flex-direction: column;
}

.map-head {
    padding: 12px 18px;
    border-bottom: 1px solid var(--border);
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-shrink: 0;
}

.map-head-title {
    font-family: var(--sans);
    font-size: 15px;
    font-weight: 700;
    letter-spacing: 1px;
    color: var(--text);
}

.map-hint {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    display: flex;
    align-items: center;
    gap: 6px;
}

.map-hint::before {
    content: '';
    width: 6px; height: 6px;
    background: var(--green);
    border-radius: 50%;
    box-shadow: 0 0 8px var(--green);
    animation: blink 1.5s infinite;
}

.map-wrap {
    flex: 1;
    display: flex;
    align-items: center;
    justify-content: center;
    overflow: auto;
    background: #040609;
    padding: 12px;
}

.map-box {
    position: relative;
    width: {{ dw }}px;
    height: {{ dh }}px;
    border-radius: 6px;
    overflow: hidden;
    border: 1px solid var(--border2);
    flex-shrink: 0;
}

#mapImg {
    width: {{ dw }}px;
    height: {{ dh }}px;
    cursor: crosshair;
    image-rendering: pixelated;
    display: block;
    opacity: 0.85;
    user-select: none;
}

#robotDot {
    position: absolute;
    width: 18px;
    height: 18px;
    background: var(--red);
    border: 3px solid #fff;
    border-radius: 50%;
    transform: translate(-50%, -50%);
    z-index: 10;
    box-shadow: 0 0 16px var(--red), 0 0 30px rgba(255,23,68,0.35);
    transition: left 0.4s ease, top 0.4s ease;
    pointer-events: none;
}

#goalDot {
    position: absolute;
    width: 14px;
    height: 14px;
    background: var(--green);
    border: 3px solid #fff;
    border-radius: 50%;
    transform: translate(-50%, -50%);
    display: none;
    z-index: 9;
    box-shadow: 0 0 16px var(--green);
    pointer-events: none;
}

#goalRing {
    position: absolute;
    width: 30px;
    height: 30px;
    border: 2px solid var(--green);
    border-radius: 50%;
    transform: translate(-50%, -50%);
    display: none;
    z-index: 8;
    pointer-events: none;
    animation: ring-pulse 1.5s infinite;
}

@keyframes ring-pulse {
    0%   { opacity:1; transform:translate(-50%,-50%) scale(1); }
    100% { opacity:0; transform:translate(-50%,-50%) scale(2.2); }
}

/* TIKLA EFEKTİ */
.click-flash {
    position: absolute;
    width: 40px; height: 40px;
    border-radius: 50%;
    background: radial-gradient(circle, rgba(0,229,255,0.5) 0%, transparent 70%);
    transform: translate(-50%, -50%) scale(0);
    pointer-events: none;
    z-index: 20;
    animation: flash 0.4s ease-out forwards;
}
@keyframes flash {
    0%   { transform:translate(-50%,-50%) scale(0); opacity:1; }
    100% { transform:translate(-50%,-50%) scale(3); opacity:0; }
}

/*  SAĞ PANEL  */
.rpanel {
    background: var(--s1);
    border-left: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    overflow: hidden;
}

.rsec {
    padding: 16px 18px;
    border-bottom: 1px solid var(--border);
}

.rsec:last-child {
    border-bottom: none;
    flex: 1;
    overflow: hidden;
    display: flex;
    flex-direction: column;
}

/* DURUM GÖSTERGESI */
.status-pill {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 10px 14px;
    background: var(--s2);
    border: 1px solid var(--border);
    border-radius: 6px;
    margin-bottom: 12px;
}

.sdot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: var(--muted);
    flex-shrink: 0;
}
.sdot.moving  { background: var(--green); box-shadow: 0 0 10px var(--green); animation: blink 1s infinite; }
.sdot.idle    { background: var(--muted); }
.sdot.stopped { background: var(--yellow); box-shadow: 0 0 8px var(--yellow); }
.sdot.error   { background: var(--red); box-shadow: 0 0 8px var(--red); }

.stext {
    font-family: var(--mono);
    font-size: 12px;
    font-weight: 700;
    letter-spacing: 1px;
}

/* DURDUR BUTONU */
.stop-btn {
    width: 100%;
    padding: 14px;
    background: rgba(255,23,68,0.08);
    border: 1px solid rgba(255,23,68,0.35);
    border-radius: 8px;
    color: var(--red);
    font-family: var(--sans);
    font-size: 16px;
    font-weight: 900;
    letter-spacing: 3px;
    cursor: pointer;
    transition: all 0.15s;
}
.stop-btn:hover {
    background: rgba(255,23,68,0.16);
    border-color: var(--red);
    box-shadow: 0 0 20px rgba(255,23,68,0.2);
}
.stop-btn:active { transform: scale(0.98); }

/* UYARILAR */
.alerts-scroll {
    flex: 1;
    overflow-y: auto;
}

.alert-item {
    display: flex;
    gap: 10px;
    padding: 9px 0;
    border-bottom: 1px solid rgba(26,37,53,0.6);
}
.alert-item:last-child { border-bottom: none; }

.abar {
    width: 3px;
    border-radius: 2px;
    flex-shrink: 0;
    align-self: stretch;
}
.abar.info    { background: var(--acc); }
.abar.success { background: var(--green); }
.abar.warning { background: var(--yellow); }
.abar.error   { background: var(--red); }

.amsg {
    font-size: 12px;
    color: var(--text);
    line-height: 1.4;
    font-family: var(--sans);
}
.atime {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    margin-top: 2px;
}

/* KAYDIRMA ÇUBUĞU */
::-webkit-scrollbar { width: 3px; }
::-webkit-scrollbar-track { background: transparent; }
::-webkit-scrollbar-thumb { background: var(--border2); border-radius: 2px; }
</style>
</head>
<body>

<!-- ÜST BAR -->
<div class="topbar">
    <div class="logo">
        <div class="logo-dot"></div>
        ROBO HAUL
    </div>
    <div class="topbar-right">
        <div class="top-stat">HIZ <span class="val" id="speedVal">0.00</span> m/s</div>
        <div class="top-stat">X <span class="val" id="topX">0.00</span></div>
        <div class="top-stat">Y <span class="val" id="topY">0.00</span></div>
        <div class="sys-tag">ÇEVRİMİÇİ</div>
    </div>
</div>

<!-- DÜZEN -->
<div class="layout">

    <!-- SOL -->
    <div class="sidebar">

        <div class="sec">
            <div class="sec-label">Robot Durumu</div>
            <div class="state-grid">
                <div class="state-row">
                    <span class="state-key">DURUM</span>
                    <span class="state-val acc" id="statusVal">Boşta</span>
                </div>
                <div class="state-row">
                    <span class="state-key">KONUM X</span>
                    <span class="state-val" id="posX">0.00</span>
                </div>
                <div class="state-row">
                    <span class="state-key">KONUM Y</span>
                    <span class="state-val" id="posY">0.00</span>
                </div>
            </div>
        </div>

        <div class="sec">
            <div class="sec-label">LiDAR</div>
            <div class="lidar-bars">
                <div class="bar-row">
                    <span class="bar-lbl">ÖN</span>
                    <div class="bar-track"><div class="bar-fill" id="barFront" style="width:100%"></div></div>
                    <span class="bar-val" id="lblFront">--</span>
                </div>
                <div class="bar-row">
                    <span class="bar-lbl">SOL</span>
                    <div class="bar-track"><div class="bar-fill" id="barLeft" style="width:100%"></div></div>
                    <span class="bar-val" id="lblLeft">--</span>
                </div>
                <div class="bar-row">
                    <span class="bar-lbl">SAĞ</span>
                    <div class="bar-track"><div class="bar-fill" id="barRight" style="width:100%"></div></div>
                    <span class="bar-val" id="lblRight">--</span>
                </div>
            </div>
        </div>

        <div class="sec">
            <div class="sec-label">Görev İstatistikleri</div>
            <div class="stats-4">
                <div class="sbox">
                    <div class="n" id="stTime">0.0</div>
                    <div class="u">SÜRE (s)</div>
                </div>
                <div class="sbox">
                    <div class="n" id="stDist">0.00</div>
                    <div class="u">MESAFE (m)</div>
                </div>
                <div class="sbox" style="grid-column:1/-1;">
                    <div class="n" id="stDone">0</div>
                    <div class="u">TAMAMLANAN GÖREV</div>
                </div>
            </div>
        </div>

        <div class="sec">
            <div class="sec-label">Hızlı Hedefler</div>
            <div id="goalsList"></div>
        </div>

        <div class="sec">
            <div class="sec-label">Hedef Ekle</div>
            <div class="add-form">
                <input class="inp" id="gName" placeholder="İsim" />
                <div class="form-row">
                    <input class="inp" id="gX" placeholder="X (m)" type="number" step="0.1" />
                    <input class="inp" id="gY" placeholder="Y (m)" type="number" step="0.1" />
                </div>
                <button class="btn-add" onclick="addGoal()">+ HEDEF KAYDET</button>
            </div>
        </div>

    </div>

    <!-- HARİTA -->
    <div class="main">
        <div class="map-card">
            <div class="map-head">
                <span class="map-head-title">FABRİKA HARİTASI</span>
                <span class="map-hint">HEDEF GÖNDERMEK İÇİN TIKLAYIN</span>
            </div>
            <div class="map-wrap">
                <div class="map-box">
                    <img id="mapImg" src="/map_image" draggable="false">
                    <div id="goalRing"></div>
                    <div id="goalDot"></div>
                    <div id="robotDot" style="left:50%;top:50%;"></div>
                </div>
            </div>
        </div>
    </div>

    <!-- SAĞ -->
    <div class="rpanel">

        <div class="rsec">
            <div class="sec-label">Görev Kontrolü</div>
            <div class="status-pill">
                <div class="sdot idle" id="sDot"></div>
                <span class="stext" id="sText">BOŞTA</span>
            </div>
            <button class="stop-btn" onclick="stopRobot()">⬛ DURDUR</button>
        </div>

        <div class="rsec">
            <div class="sec-label">Uyarılar</div>
            <div class="alerts-scroll" id="alertsBox">
                <div class="alert-item">
                    <div class="abar info"></div>
                    <div>
                        <div class="amsg">Sistem hazır</div>
                        <div class="atime">--:--:--</div>
                    </div>
                </div>
            </div>
        </div>

    </div>
</div>

<script>
const SCALE   = {{ scale }};
const MAP_H   = {{ map_h }};
const MAP_RES = {{ map_res }};
const MAP_OX  = {{ map_ox }};
const MAP_OY  = {{ map_oy }};

const mapImg   = document.getElementById("mapImg");
const robotDot = document.getElementById("robotDot");
const goalDot  = document.getElementById("goalDot");
const goalRing = document.getElementById("goalRing");

let goals = [];
let lastAlertCount = 0;

//  Hedefleri yükle 
fetch("/get_goals").then(r=>r.json()).then(d => {
    goals = d.goals;
    renderGoals();
});

function renderGoals() {
    const el = document.getElementById("goalsList");
    el.innerHTML = "";
    goals.forEach(g => {
        const div = document.createElement("div");
        div.className = "qgoal";
        div.innerHTML = `
            <div class="qdot"></div>
            <span>${g.name}</span>
            <span class="qcoords">${g.x.toFixed(1)}, ${g.y.toFixed(1)}</span>
        `;
        div.onclick = () => sendDirect(g.x, g.y, g.name);
        el.appendChild(div);
    });
}

function addGoal() {
    const name = document.getElementById("gName").value.trim();
    const x = parseFloat(document.getElementById("gX").value);
    const y = parseFloat(document.getElementById("gY").value);
    if (!name || isNaN(x) || isNaN(y)) return;
    fetch("/add_goal", {
        method:"POST",
        headers:{"Content-Type":"application/json"},
        body: JSON.stringify({name, x, y})
    }).then(r=>r.json()).then(d => {
        goals = d.goals;
        renderGoals();
        document.getElementById("gName").value = "";
        document.getElementById("gX").value = "";
        document.getElementById("gY").value = "";
    });
}

mapImg.addEventListener("click", function(e) {
    const rect   = mapImg.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;
    const pixelX = clickX / SCALE;
    const pixelY = clickY / SCALE;

    const mapX = MAP_OX + pixelX * MAP_RES;
    const mapY = MAP_OY + (MAP_H - pixelY) * MAP_RES;

 
    goalDot.style.left    = clickX + "px";
    goalDot.style.top     = clickY + "px";
    goalDot.style.display = "block";

    goalRing.style.left    = clickX + "px";
    goalRing.style.top     = clickY + "px";
    goalRing.style.display = "block";

    const flash = document.createElement("div");
    flash.className = "click-flash";
    flash.style.left = clickX + "px";
    flash.style.top  = clickY + "px";
    mapImg.parentElement.appendChild(flash);
    setTimeout(() => flash.remove(), 500);

    fetch("/send_goal", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({pixel_x: pixelX, pixel_y: pixelY})
    }).then(r=>r.json()).then(d => {
        addAlert(`Hedef → (${d.x.toFixed(2)}, ${d.y.toFixed(2)})`, "info");
    });
});

function sendDirect(x, y, name) {
    fetch("/send_goal_direct", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({x, y, name})
    }).then(r=>r.json()).then(d => {
        addAlert(`Hızlı → ${name || ""} (${d.x.toFixed(2)}, ${d.y.toFixed(2)})`, "info");
    });
}

function stopRobot() {
    fetch("/stop", {method:"POST"})
    .then(r=>r.json())
    .then(d => updateStatusUI(d.status));
}

function updatePose() {
    fetch("/robot_pose").then(r=>r.json()).then(d => {
        robotDot.style.left = (d.pixel_x * SCALE) + "px";
        robotDot.style.top  = (d.pixel_y * SCALE) + "px";
        document.getElementById("posX").textContent = d.x.toFixed(2);
        document.getElementById("posY").textContent = d.y.toFixed(2);
        document.getElementById("topX").textContent = d.x.toFixed(2);
        document.getElementById("topY").textContent = d.y.toFixed(2);
    }).catch(()=>{});
}

function updateStatus() {
    fetch("/full_status").then(r=>r.json()).then(d => {
        updateStatusUI(d.status);


        document.getElementById("speedVal").textContent = (d.speed || 0).toFixed(2);

        document.getElementById("stTime").textContent = d.stats.elapsed.toFixed(1);
        document.getElementById("stDist").textContent = d.stats.distance.toFixed(2);
        document.getElementById("stDone").textContent = d.stats.completed;

    
        updateLidar(d.lidar);

        if (d.new_alerts && d.new_alerts.length > 0) {
            d.new_alerts.forEach(a => addAlert(a.msg, a.level, a.time));
        }
    }).catch(()=>{});
}

function updateLidar(l) {
    if (!l) return;
    const MAX = 3.0;
    ["front","left","right"].forEach(dir => {
        const val = l[dir] || 0;
        const pct = Math.min(100, (val / MAX) * 100);
        const bar = document.getElementById("bar" + dir.charAt(0).toUpperCase() + dir.slice(1));
        const lbl = document.getElementById("lbl" + dir.charAt(0).toUpperCase() + dir.slice(1));
        bar.style.width = pct + "%";
        bar.className = "bar-fill";
        if (val < 0.55) bar.classList.add("danger");
        else if (val < 1.0) bar.classList.add("warn");
        lbl.textContent = val > 90 ? "--" : val.toFixed(2) + "m";
    });
}

function updateStatusUI(status) {
    document.getElementById("statusVal").textContent = status;
    document.getElementById("sText").textContent = status.toUpperCase();
    const dot = document.getElementById("sDot");
    dot.className = "sdot";
    if (status === "Hareket Ediyor") dot.classList.add("moving");
    else if (status === "Durduruldu") dot.classList.add("stopped");
    else if (status.includes("hata") || status.includes("red")) dot.classList.add("error");
    else dot.classList.add("idle");
}

function addAlert(msg, level, t) {
    const box  = document.getElementById("alertsBox");
    const item = document.createElement("div");
    item.className = "alert-item";
    item.innerHTML = `
        <div class="abar ${level||'info'}"></div>
        <div>
            <div class="amsg">${msg}</div>
            <div class="atime">${t || new Date().toLocaleTimeString()}</div>
        </div>
    `;
    box.prepend(item);
    while (box.children.length > 40) box.removeChild(box.lastChild);
}

setInterval(updatePose,   400);
setInterval(updateStatus, 500);
updatePose();
updateStatus();
</script>
</body>
</html>
"""



# Flask Rotaları
last_alert_idx = 0
@app.route("/")
def index():
    return render_template_string(
        HTML,
        dw=DISPLAY_WIDTH,
        dh=DISPLAY_HEIGHT,
        scale=DISPLAY_SCALE,
        map_h=MAP_HEIGHT,
        map_res=MAP_RESOLUTION,
        map_ox=MAP_ORIGIN_X,
        map_oy=MAP_ORIGIN_Y,
    )

@app.route("/map_image")
def map_image():
    # Harita görüntüsünü gönder
    return send_file(MAP_IMAGE)

@app.route("/robot_pose")
def robot_pose():
    # Robot konumunu harita piksel koordinatlarına çevir
    x = node.robot_x
    y = node.robot_y
    px = (x - MAP_ORIGIN_X) / MAP_RESOLUTION
    py = MAP_HEIGHT - ((y - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return jsonify({"x": x, "y": y, "pixel_x": px, "pixel_y": py})

@app.route("/send_goal", methods=["POST"])
def send_goal():
    # Haritaya tıklama noktasını gerçek koordinatlara çevir ve hedef olarak gönder
    data   = request.get_json()
    px     = float(data["pixel_x"])
    py     = float(data["pixel_y"])
    map_x  = MAP_ORIGIN_X + px * MAP_RESOLUTION
    map_y  = MAP_ORIGIN_Y + (MAP_HEIGHT - py) * MAP_RESOLUTION
    node.send_goal(map_x, map_y, "Harita Tıklaması")
    return jsonify({"x": map_x, "y": map_y})

@app.route("/send_goal_direct", methods=["POST"])
def send_goal_direct():
    # Gerçek koordinatlarla doğrudan hedef gönder
    data = request.get_json()
    x    = float(data["x"])
    y    = float(data["y"])
    name = data.get("name", "Hızlı Hedef")
    node.send_goal(x, y, name)
    return jsonify({"x": x, "y": y})

@app.route("/stop", methods=["POST"])
def stop():
    # Robotu durdur
    node.stop_robot()
    return jsonify({"status": node.status})

@app.route("/full_status")
def full_status():
    # Tüm bilgileri tek seferde gönder
    global last_alert_idx
    new_alerts     = node.alerts[last_alert_idx:]
    last_alert_idx = len(node.alerts)
    return jsonify({
        "status":     node.status,
        "speed":      node.robot_speed,
        "stats":      node.get_stats(),
        "lidar": {
            "front": round(node.front_dist, 2),
            "left":  round(node.left_dist,  2),
            "right": round(node.right_dist, 2),
        },
        "new_alerts": new_alerts,
    })

@app.route("/get_goals")
def get_goals():
    return jsonify({"goals": node.saved_goals})

@app.route("/add_goal", methods=["POST"])
def add_goal():
    # Listeye yeni hedef ekle
    data = request.get_json()
    node.saved_goals.append({
        "name": data["name"],
        "x":    float(data["x"]),
        "y":    float(data["y"]),
    })
    return jsonify({"goals": node.saved_goals})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)