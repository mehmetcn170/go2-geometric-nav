#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import threading
import time

class Robot(Node):
    def __init__(self):
        super().__init__('daire_robot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom/ground_truth', self._odom_cb, 10)
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
            yaw_rad = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))
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
        time.sleep(0.2)

    def don(self, hedef_deg):
        timeout = time.time() + 6
        while time.time() < timeout:
            x, y, yaw = self.pose()
            ad = hedef_deg - yaw
            while ad > 180:  ad -= 360
            while ad < -180: ad += 360
            if abs(ad) < 5:
                self.dur()
                return
            az = max(min(ad * 0.02, 0.7), -0.7)
            self.vel(0.0, az)
            time.sleep(0.05)
        self.dur()

    def git(self, hx, hy, tol=0.5):
        """Hedefe git — durma yok, akici gecis."""
        x, y, _ = self.pose()
        hedef_deg = math.degrees(math.atan2(hy-y, hx-x))
        self.don(hedef_deg)

        timeout = time.time() + 20
        while time.time() < timeout:
            x, y, yaw = self.pose()
            dx, dy = hx-x, hy-y
            dist = math.sqrt(dx*dx+dy*dy)

            if dist < tol:
                # Durmadan bir sonraki waypointte devam et
                return

            ta_deg = math.degrees(math.atan2(dy, dx))
            ad = ta_deg - yaw
            while ad > 180:  ad -= 360
            while ad < -180: ad += 360

            if abs(ad) > 25:
                self.don(ta_deg)
                continue

            # Sabit hiz — durma yok
            lx = 0.35
            az = ad * 0.025
            self.vel(lx, az)
            time.sleep(0.05)
        self.dur()

def main():
    rclpy.init()
    robot = Robot()
    t = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    t.start()

    print('Odom bekleniyor...')
    while not robot.ready:
        time.sleep(0.1)
    time.sleep(2)

    # Daire parametreleri
    r        = 3.0   # Yaricap
    n        = 12    # Az waypoint = daha akici
    num_laps = 3

    waypoints = []
    for i in range(n):
        angle = 2 * math.pi * i / n
        waypoints.append((r * math.cos(angle), r * math.sin(angle)))

    print(f'Daire: r={r}m | {n} waypoint | {num_laps} tur')

    print('Baslangic noktasina gidiliyor...')
    robot.git(waypoints[0][0], waypoints[0][1], tol=0.3)
    time.sleep(0.5)

    for lap in range(num_laps):
        print(f'\n=== TUR {lap+1}/{num_laps} ===')
        for i, (wx, wy) in enumerate(waypoints[1:] + [waypoints[0]], 1):
            print(f'  WP {i}/{n}: ({wx:.1f}, {wy:.1f})')
            robot.git(wx, wy, tol=0.5)
        print(f'TUR {lap+1} TAMAM!')

    print('\nTUM TURLAR TAMAMLANDI!')
    robot.dur()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
