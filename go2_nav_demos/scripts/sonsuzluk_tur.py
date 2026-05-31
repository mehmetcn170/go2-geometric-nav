#!/usr/bin/env python3
"""
sonsuzluk_tur.py  —  Go2 robotu ∞ (sonsuzluk / lemniskat) yörüngede döner.

Yörünge: Bernoulli Lemniskatı
    x(t) = a · cos(t) / (1 + sin²(t))
    y(t) = a · sin(t)·cos(t) / (1 + sin²(t))

Parametreler (main içinde ayarlanabilir):
    a        : lemniskat ölçek katsayısı (metre)
    N        : waypoint sayısı (büyük → pürüzsüz)
    num_laps : tur sayısı (her tur = tam bir ∞)
    speed    : ileri hız (m/s)

Kullanım:
    ros2 run go2_nav_demos sonsuzluk_tur
    veya
    python3 sonsuzluk_tur.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import threading
import time


class Robot(Node):
    def __init__(self):
        super().__init__('sonsuzluk_robot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/odom/ground_truth', self._odom_cb, 10
        )
        self.x = 0.0
        self.y = 0.0
        self.yaw_deg = 0.0
        self.ready = False
        self._lock = threading.Lock()

    def _odom_cb(self, msg):
        with self._lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw_rad = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y ** 2 + q.z ** 2)
            )
            self.yaw_deg = math.degrees(yaw_rad)
            self.ready = True

    def pose(self):
        with self._lock:
            return self.x, self.y, self.yaw_deg

    def vel(self, lx, az):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.angular.z = float(az)
        self.pub.publish(msg)

    def dur(self):
        self.vel(0, 0)
        time.sleep(0.3)

    def don(self, hedef_deg, tol=3.0, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            _, _, yaw = self.pose()
            ad = hedef_deg - yaw
            while ad > 180:
                ad -= 360
            while ad < -180:
                ad += 360
            if abs(ad) < tol:
                self.dur()
                return
            az = max(min(ad * 0.025, 0.8), -0.8)
            self.vel(0.0, az)
            time.sleep(0.05)
        self.dur()

    def git(self, hx, hy, v=0.30, tol=0.18, timeout=60.0):
        x, y, _ = self.pose()
        hedef_deg = math.degrees(math.atan2(hy - y, hx - x))
        self.don(hedef_deg)

        t0 = time.time()
        while time.time() - t0 < timeout:
            x, y, yaw = self.pose()
            dx, dy = hx - x, hy - y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < tol:
                self.dur()
                return

            ta_deg = math.degrees(math.atan2(dy, dx))
            ad = ta_deg - yaw
            while ad > 180:
                ad -= 360
            while ad < -180:
                ad += 360

            if abs(ad) > 25:
                self.don(ta_deg)
                continue

            lx = min(v, dist * 0.6)
            az = ad * 0.025
            self.vel(lx, az)
            time.sleep(0.05)
        self.dur()


# ------------------------------------------------------------------ #
#  Lemniskat waypoint üretici
# ------------------------------------------------------------------ #
def sonsuzluk_waypoints(a=1.8, N=72, cx=0.0, cy=0.0):
    """
    Bernoulli Lemniskatı:
        x = a·cos(t) / (1+sin²(t))
        y = a·sin(t)·cos(t) / (1+sin²(t))
    t ∈ [0, 2π) → tam bir ∞ çizer.

    a  : ölçek (yaklaşık maksimum x genişliği = a)
    N  : waypoint sayısı
    """
    pts = []
    for i in range(N):
        t = 2 * math.pi * i / N
        denom = 1 + math.sin(t) ** 2
        x = cx + a * math.cos(t) / denom
        y = cy + a * math.sin(t) * math.cos(t) / denom
        pts.append((x, y))
    return pts


# ------------------------------------------------------------------ #
#  Ana program
# ------------------------------------------------------------------ #
def main():
    # ---- Parametreler ----
    a        = 1.8    # lemniskat ölçeği (m)  — 5x5 alana sığar
    N        = 72     # waypoint sayısı (daha pürüzsüz ∞ için artırın)
    num_laps = 3
    speed    = 0.30   # m/s  (geçiş noktasında yavaş)
    # ----------------------

    rclpy.init()
    robot = Robot()

    t = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    t.start()

    print('Odometry bekleniyor...')
    while not robot.ready:
        time.sleep(0.1)
    time.sleep(2)

    waypoints = sonsuzluk_waypoints(a=a, N=N)
    print(f'Sonsuzluk yörüngesi hazır: a={a} m, {N} waypoint, {num_laps} tur')
    print(f'  X aralığı: [{min(p[0] for p in waypoints):.2f}, {max(p[0] for p in waypoints):.2f}]')
    print(f'  Y aralığı: [{min(p[1] for p in waypoints):.2f}, {max(p[1] for p in waypoints):.2f}]')

    # Başlangıç noktasına git (merkez = (0,0) — çapraz geçiş noktası)
    print('\nBaşlangıç noktasına gidiliyor: (0, 0)')
    robot.git(0.0, 0.0, v=speed)
    time.sleep(1)

    for lap in range(num_laps):
        print(f'\n=== TUR {lap + 1}/{num_laps} ===')
        for i, (wx, wy) in enumerate(waypoints, 1):
            print(f'  WP {i:02d}/{N}  ({wx:.2f}, {wy:.2f})')
            robot.git(wx, wy, v=speed, tol=0.15)
        print(f'=== TUR {lap + 1} TAMAMLANDI ===')

    print('\nBİTTİ!')
    robot.dur()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
