#!/usr/bin/env python3
"""
kare_tur.py  —  Go2 robotu 5x5 m kare yolda 3 tur atar.

Kullanım:
    ros2 run go2_nav_demos kare_tur
    veya
    python3 kare_tur.py
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
        super().__init__('kare_robot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/odom/ground_truth', self._odom_cb, 10
        )
        self.x = 0.0
        self.y = 0.0
        self.yaw_deg = 0.0
        self.ready = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    #  Odometry callback
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    #  Yardımcı metodlar
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    #  Yönelim: hedef_deg açısına dön
    # ------------------------------------------------------------------ #
    def don(self, hedef_deg):
        print(f'  ↻ Dönüş: {hedef_deg:.0f}°')
        timeout = time.time() + 8
        while time.time() < timeout:
            x, y, yaw = self.pose()
            ad = hedef_deg - yaw
            while ad > 180:
                ad -= 360
            while ad < -180:
                ad += 360
            if abs(ad) < 3:
                self.dur()
                return
            az = max(min(ad * 0.02, 0.7), -0.7)
            self.vel(0.0, az)
            time.sleep(0.05)
        self.dur()

    # ------------------------------------------------------------------ #
    #  İleri git: (hx, hy) noktasına git
    # ------------------------------------------------------------------ #
    def git(self, hx, hy):
        print(f'  → Hedef: ({hx:.1f}, {hy:.1f})')
        x, y, _ = self.pose()
        hedef_deg = math.degrees(math.atan2(hy - y, hx - x))
        self.don(hedef_deg)

        timeout = time.time() + 60
        while time.time() < timeout:
            x, y, yaw = self.pose()
            dx, dy = hx - x, hy - y
            dist = math.sqrt(dx * dx + dy * dy)
            print(f'    ({x:.2f},{y:.2f})  dist={dist:.2f} m')

            if dist < 0.2:
                self.dur()
                print('  ✓ Hedefe ulaşıldı!')
                return

            ta_deg = math.degrees(math.atan2(dy, dx))
            ad = ta_deg - yaw
            while ad > 180:
                ad -= 360
            while ad < -180:
                ad += 360

            if abs(ad) > 20:
                self.don(ta_deg)
                continue

            lx = min(0.4, dist * 0.5)
            az = ad * 0.02
            self.vel(lx, az)
            time.sleep(0.05)
        self.dur()


# ------------------------------------------------------------------ #
#  Ana program
# ------------------------------------------------------------------ #
def main():
    rclpy.init()
    robot = Robot()

    t = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    t.start()

    print('Odometry bekleniyor...')
    while not robot.ready:
        time.sleep(0.1)
    time.sleep(2)

    # 5x5 kare köşeleri (merkez = 0,0)
    koseler = [
        (-2.5, -2.5),
        ( 2.5, -2.5),
        ( 2.5,  2.5),
        (-2.5,  2.5),
        (-2.5, -2.5),   # başlangıca dön
    ]
    num_laps = 3

    print('Başlangıç köşesine gidiliyor...')
    robot.git(-2.5, -2.5)
    time.sleep(1)

    for lap in range(num_laps):
        print(f'\n=== TUR {lap + 1}/{num_laps} ===')
        for kose in koseler[1:]:
            robot.git(kose[0], kose[1])
        print(f'=== TUR {lap + 1} TAMAMLANDI ===')

    print('\nBİTTİ!')
    robot.dur()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
