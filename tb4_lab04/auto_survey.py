#!/usr/bin/env python3
"""Node tu lai khao sat de SLAM Toolbox ve ban do (thay cho teleop ban phim).

Doc /scan, bam tuong ben phai, publish /cmd_vel. Tu dung khi robot da di du quang duong va quay
ve gan diem xuat phat (hoac het thoi gian toi da). Ctrl+C luon dung robot an toan.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from tb4_lab04.wall_follow import Params, compute_cmd


class AutoSurvey(Node):
    def __init__(self):
        super().__init__('auto_survey')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('speed', 0.18)
        self.declare_parameter('max_ang', 0.8)
        self.declare_parameter('target_dist', 0.55)
        self.declare_parameter('start_delay', 3.0)        # giay cho he thong on dinh
        self.declare_parameter('max_duration', 900.0)     # giay
        self.declare_parameter('min_path', 15.0)          # met di toi thieu truoc khi xet quay ve
        self.declare_parameter('return_radius', 0.7)      # met
        self.declare_parameter('min_excursion', 2.5)      # met: phai tung di xa diem xuat phat it nhat the nay
        self.declare_parameter('wall_memory', 6.0)        # giay: thoi gian con coi la 'vua bam tuong'

        g = self.get_parameter
        self.params = Params(speed=g('speed').value, max_ang=g('max_ang').value,
                             target_dist=g('target_dist').value)
        self.start_delay = g('start_delay').value
        self.max_duration = g('max_duration').value
        self.min_path = g('min_path').value
        self.return_radius = g('return_radius').value
        self.min_excursion = g('min_excursion').value
        self.wall_memory = g('wall_memory').value

        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic').value, 10)
        self.create_subscription(LaserScan, g('scan_topic').value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic').value, self._on_odom, qos_profile_sensor_data)

        self.scan = None
        self.start_xy = None
        self.last_xy = None
        self.path = 0.0
        self.max_dist = 0.0
        self.last_wall_t = -1e9
        self.wall_run_start = None   # thoi diem bat dau chuoi bam tuong lien tuc
        self.anchor_xy = None        # diem bat dau bam tuong (dung de phat hien vong kin)
        self.anchor_path = 0.0
        self.anchor_max = 0.0
        self.t0 = self.get_clock().now()
        self.done = False
        self.last_state = ''
        self.create_timer(0.1, self._step)
        self.create_timer(5.0, self._report)
        self.get_logger().info('auto_survey: cho scan/odom roi bat dau bam tuong ben phai')

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.start_xy is None:
            self.start_xy = xy
        if self.last_xy is not None:
            step = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            self.path += step
            if self.anchor_xy is not None:
                self.anchor_path += step
        self.last_xy = xy
        self.max_dist = max(self.max_dist, math.hypot(xy[0] - self.start_xy[0], xy[1] - self.start_xy[1]))
        if self.anchor_xy is not None:
            self.anchor_max = max(self.anchor_max, math.hypot(xy[0] - self.anchor_xy[0], xy[1] - self.anchor_xy[1]))

    def _elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _finish(self, reason):
        self.done = True
        for _ in range(5):
            self._stop()
        self.get_logger().info(f'auto_survey: DUNG - {reason} (quang duong {self.path:.1f} m). '
                               'Luu ban do: scripts/save_map.sh')

    def _step(self):
        if self.done:
            return
        if self._elapsed() < self.start_delay or self.scan is None or self.start_xy is None:
            return
        if self._elapsed() > self.max_duration:
            self._finish('het thoi gian toi da')
            return
        if (self.anchor_xy is not None and self.anchor_path > self.min_path and self.anchor_max >= self.min_excursion
                and self.last_xy is not None
                and math.hypot(self.last_xy[0] - self.anchor_xy[0], self.last_xy[1] - self.anchor_xy[1]) < self.return_radius):
            self._finish('da khep kin vong bam tuong (quay lai diem bat dau bam tuong)')
            return
        s = self.scan
        rmax = s.range_max if math.isfinite(s.range_max) and s.range_max > 0 else 8.0
        recent = (self._elapsed() - self.last_wall_t) < self.wall_memory
        lin, ang, state = compute_cmd(list(s.ranges), s.angle_min, s.angle_increment, self.params, rmax, recent)
        if state == 'bam_tuong':
            self.last_wall_t = self._elapsed()
            if self.wall_run_start is None:
                self.wall_run_start = self._elapsed()
            elif self.anchor_xy is None and self._elapsed() - self.wall_run_start > 5.0 and self.last_xy is not None:
                self.anchor_xy = self.last_xy      # da bam tuong on dinh >5 s: dat moc vong kin
        else:
            self.wall_run_start = None
        self.last_state = state
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_pub.publish(cmd)

    def _report(self):
        if not self.done and self.scan is not None:
            self.get_logger().info(f'trang thai={self.last_state}, quang duong={self.path:.1f} m, xa xuat phat toi da={self.max_dist:.1f} m, '
                                   f'moc bam tuong={"co" if self.anchor_xy else "chua"}, '
                                   f't={self._elapsed():.0f}s')


def main():
    rclpy.init()
    node = AutoSurvey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
