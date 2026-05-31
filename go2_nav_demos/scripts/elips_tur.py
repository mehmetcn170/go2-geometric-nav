#!/usr/bin/env python3
"""
elips_tur.py  —  Go2 robotu elips (oval) yörüngede döner.

Parametreler (main içinde ayarlanabilir):
    a        : elipsin X yarıçapı (metre)
    b        : elipsin Y yarıçapı (metre)
    num_laps : tur sayısı
    N        : yörünge üzerindeki waypoint sayısı
    speed    : ileri hız (m/s)

Kullanım:
    ros2 run go2_nav_demos elips_tur
    veya
    python3 elips_tur.py
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
        super().__init__('elips_robot')
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

    def git(self, hx, hy, v=0.35, tol=0.20, timeout=60.0):
        """Hedefe git; yaklaşırken hızı yumuşat."""
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
#  Elips waypoint üretici
# ------------------------------------------------------------------ #
def elips_waypoints(a=2.0, b=1.2, N=36, cx=0.0, cy=0.0):
    """
    a  : X yarıçapı
    b  : Y yarıçapı
    N  : waypoint sayısı (büyük → pürüzsüz)
    """
    pts = []
    for i in range(N):
        theta = 2 * math.pi * i / N
        pts.append((cx + a * math.cos(theta),
                    cy + b * math.sin(theta)))
    return pts


# ------------------------------------------------------------------ #
#  Ana program
# ------------------------------------------------------------------ #
def main():
    # ---- Parametreler ----
    a        = 2.0    # X yarıçapı (m)  — duvarlardan ~0.5 m boşluk
    b        = 1.2    # Y yarıçapı (m)
    N        = 36     # waypoint sayısı
    num_laps = 3
    speed    = 0.35   # m/s
    # ----------------------

    rclpy.init()
    robot = Robot()

    t = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    t.start()

    print('Odometry bekleniyor...')
    while not robot.ready:
        time.sleep(0.1)
    time.sleep(2)

    waypoints = elips_waypoints(a=a, b=b, N=N)
    print(f'Elips hazır: a={a} m, b={b} m, {N} waypoint, {num_laps} tur')

    # Başlangıç noktasına git
    print('Başlangıç noktasına gidiliyor...')
    robot.git(waypoints[0][0], waypoints[0][1], v=speed)
    time.sleep(1)

    for lap in range(num_laps):
        print(f'\n=== TUR {lap + 1}/{num_laps} ===')
        for i, (wx, wy) in enumerate(waypoints[1:] + [waypoints[0]], 1):
            print(f'  WP {i}/{N}  ({wx:.2f}, {wy:.2f})')
            robot.git(wx, wy, v=speed, tol=0.18)
        print(f'=== TUR {lap + 1} TAMAMLANDI ===')

    print('\nBİTTİ!')
    robot.dur()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
