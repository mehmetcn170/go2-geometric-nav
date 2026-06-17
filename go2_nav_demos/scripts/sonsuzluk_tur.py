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
        super().__init__('sonsuzluk_robot')
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

    def git(self, hx, hy, tol=0.35):
        x, y, _ = self.pose()
        hedef_deg = math.degrees(math.atan2(hy-y, hx-x))
        self.don(hedef_deg)
        timeout = time.time() + 20
        while time.time() < timeout:
            x, y, yaw = self.pose()
            dx, dy = hx-x, hy-y
            dist = math.sqrt(dx*dx+dy*dy)
            if dist < tol:
                return
            ta_deg = math.degrees(math.atan2(dy, dx))
            ad = ta_deg - yaw
            while ad > 180:  ad -= 360
            while ad < -180: ad += 360
            if abs(ad) > 25:
                self.don(ta_deg)
                continue
            self.vel(0.35, ad * 0.025)
            time.sleep(0.05)
        self.dur()

def lemniscate(t, a=3.5):
    denom = 1 + math.sin(t)**2
    x = a * math.cos(t) / denom
    y = a * math.sin(t) * math.cos(t) / denom
    return x, y

def main():
    rclpy.init()
    robot = Robot()
    t = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    t.start()

    print('Odom bekleniyor...')
    while not robot.ready:
        time.sleep(0.1)
    time.sleep(2)

    a        = 3.5
    n        = 32
    num_laps = 3

    # pi/2'den baslat — ilk nokta (0,0)
    waypoints = []
    for i in range(n):
        t_val = math.pi/2 + 2 * math.pi * i / n
        wx, wy = lemniscate(t_val, a)
        waypoints.append((wx, wy))

    print(f'Sonsuzluk rotasi | {n} waypoint | {num_laps} tur')
    print(f'Ilk nokta: ({waypoints[0][0]:.2f}, {waypoints[0][1]:.2f}) — robot burada basliyor')

    # Robot zaten (0,0)da, direkt tura basla
    for lap in range(num_laps):
        print(f'\n=== TUR {lap+1}/{num_laps} ===')
        for i, (wx, wy) in enumerate(waypoints, 1):
            dist_to_center = math.sqrt(wx**2 + wy**2)
            tol = 0.5 if dist_to_center < 0.5 else 0.35
            print(f'  WP {i}/{n}: ({wx:.1f}, {wy:.1f})')
            robot.git(wx, wy, tol=tol)
        print(f'TUR {lap+1} TAMAM!')

    print('\nTUM TURLAR TAMAMLANDI!')
    robot.dur()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
