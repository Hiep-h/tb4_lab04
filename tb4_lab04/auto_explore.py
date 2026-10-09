#!/usr/bin/env python3
"""Node tu kham pha de SLAM Toolbox ve ban do nhanh (thay cho lai tay / bam tuong).

Doc /map (tu slam_toolbox) + TF (map -> base_link) + /scan, chon bien (ranh gioi trong/chua biet) gan nhat theo
duong di that, lap duong tranh vat can va bam duong bang pure pursuit, publish /cmd_vel. Khi het bien thi dung robot
va tu luu ban do (map.yaml + map.pgm). Ctrl+C luon dung robot an toan.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from tb4_lab04.explorer import Explorer
from tb4_lab04.map_saver import MapSaver
from tb4_lab04.wall_follow import sector_min


def grid_from_msg(msg):
    """OccupancyGrid -> (mang int16 (h, w), do phan giai, goc (ox, oy))."""
    grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    return grid, float(msg.info.resolution), (msg.info.origin.position.x, msg.info.origin.position.y)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoExplore(Node):
    def __init__(self):
        super().__init__('auto_explore')
        for name, default in [('scan_topic', '/scan'), ('map_topic', '/map'), ('cmd_topic', '/cmd_vel'),
                              ('map_frame', 'map'), ('base_frame', 'base_link'), ('vmax', 0.45), ('wmax', 1.8),
                              ('inflate_m', 0.30), ('clearance_m', 0.25), ('replan_period', 1.5),
                              ('start_delay', 3.0), ('max_duration', 1800.0), ('save_on_finish', True), ('min_known_cells', 2000),
                              ('map_dir', '~/tb4_ws/src/tb4_lab04/maps')]:
            self.declare_parameter(name, default)
        g = self.get_parameter
        self.explorer = Explorer(vmax=g('vmax').value, wmax=g('wmax').value, inflate_m=g('inflate_m').value,
                                 clearance_m=g('clearance_m').value, replan_period=g('replan_period').value)
        self.map_frame, self.base_frame = g('map_frame').value, g('base_frame').value
        self.start_delay, self.max_duration = g('start_delay').value, g('max_duration').value
        self.save_on_finish = g('save_on_finish').value
        self.min_known_cells = g('min_known_cells').value
        self.empty_warned_t = -1e9
        self.saver = MapSaver(g('map_dir').value)

        self.pub = self.create_publisher(Twist, g('cmd_topic').value, 10)
        self.create_subscription(LaserScan, g('scan_topic').value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, g('map_topic').value, self._on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.front = None
        self.scan_stats = (0, 0)       # (so tia huu han, tong so tia)
        self.blind_since = None
        self.done = False
        self.t0 = self.get_clock().now()
        self.last_state = 'cho'
        self.create_timer(0.1, self._step)
        self.create_timer(5.0, self._report)
        self.get_logger().info('auto_explore: cho /map, TF va /scan roi bat dau kham pha')

    def _now(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _on_scan(self, msg):
        rmax = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0 else 8.0
        rs = list(msg.ranges)
        self.front = sector_min(rs, msg.angle_min, msg.angle_increment, -25, 25, rmax)
        self.scan_stats = (sum(1 for r in rs if math.isfinite(r)), len(rs))

    def _on_map(self, msg):
        self.map = grid_from_msg(msg)

    def _pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        p, q = t.transform.translation, t.transform.rotation
        return p.x, p.y, yaw_from_quat(q)

    def _stop(self):
        self.pub.publish(Twist())

    def _finish(self, why):
        self.done = True
        for _ in range(5):
            self._stop()
        self.get_logger().info('auto_explore: HOAN TAT - %s (%.0f s)' % (why, self._now()))
        if self.save_on_finish:
            self.get_logger().info('auto_explore: dang luu ban do...')
            self.saver.save_blocking()
            self.get_logger().info('auto_explore: ' + self.saver.last)

    def _step(self):
        if self.done:
            return
        t = self._now()
        if t < self.start_delay or self.map is None:
            return
        if t > self.max_duration:
            self._finish('het thoi gian toi da')
            return
        pose = self._pose()
        if pose is None:
            return
        grid, res, origin = self.map
        v, w, state = self.explorer.update(t, grid, res, origin, pose, self.front)
        self.last_state = state
        if state == 'done':
            known_cells = int(np.count_nonzero(grid != -1))
            if known_cells < self.min_known_cells:
                # ban do SLAM con qua nho: khong ket luan "da xong" va khong luu ban do rong
                if t - self.empty_warned_t > 10.0:
                    self.empty_warned_t = t
                    self.get_logger().warn('Ban do SLAM gan nhu RONG (%d o da biet < %d): slam_toolbox chua tao ban do '
                                           'tu /scan. Khong ket thuc, khong luu ban do.' % (known_cells, self.min_known_cells))
                self.explorer.state = 'explore'
                self.explorer.no_goal_count = 0
                self._stop()
                return
            self._finish('khong con vung chua biet nao toi duoc')
            return
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)

    def _report(self):
        if self.done:
            return
        fin, tot = self.scan_stats
        known = '?'
        if self.map is not None:
            grid = self.map[0]
            known = '%d%%' % (100 * np.count_nonzero(grid != -1) // max(1, grid.size))
        pose = self._pose()
        self.get_logger().info('trang thai=%s, t=%.0fs, vi tri=%s, o da biet=%s, tia LiDAR huu han=%d/%d' % (
            self.last_state, self._now(), 'chua co TF' if pose is None else '(%.1f, %.1f)' % pose[:2], known, fin, tot))
        # canh bao neu LiDAR khong thay gi (tat ca tia = inf)
        if tot > 0 and fin / tot < 0.02:
            self.blind_since = self.blind_since or self._now()
            if self._now() - self.blind_since > 15.0:
                self.get_logger().warn('LiDAR gan nhu KHONG thay vat can (tia huu han %d/%d) - ban do se rong' % (fin, tot))
        else:
            self.blind_since = None


def main():
    rclpy.init()
    node = AutoExplore()
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
