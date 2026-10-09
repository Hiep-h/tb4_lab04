#!/usr/bin/env python3
"""Bang dieu khien TurtleBot 4 bang thanh truot (Tkinter), thay cho teleop ban phim tren terminal.

Chay:  ros2 run tb4_lab04 control_panel
       ros2 run tb4_lab04 control_panel --ros-args -p max_linear:=0.3 -p max_angular:=1.0

- Thanh "Tien / Lui" -> van toc thang (m/s); thanh "Xoay tron" -> van toc quay (rad/s).
- TurtleBot 4 la robot hai banh vi sai nen khong co truot ngang nhu xe Mecanum.
- Phim: SPACE = phanh khan cap, mui ten len/xuong = tang/giam toc, trai/phai = quay.
- "Chong va cham": tu chan lenh tien khi vat can phia truoc gan hon nguong (dung /scan).
- "Tu lai (bam tuong)": robot tu bam tuong ben phai bang LiDAR de khao sat ve ban do.
"""
import math
import os
import subprocess
import threading
import time
import tkinter as tk

from tb4_lab04.map_saver import MapSaver  # noqa: F401  (giu ten cu de tuong thich)
from tb4_lab04.wall_follow import Params, compute_cmd

FONT = ('DejaVu Sans', 10)
FONT_B = ('DejaVu Sans', 10, 'bold')


class Panel:
    def __init__(self, root, bridge, max_linear=0.3, max_angular=1.0, stop_dist=0.30, saver=None, autosave_period=120.0):
        self.root, self.bridge = root, bridge
        self.saver = saver
        self.autosave_period = autosave_period
        self._last_autosave = time.time()
        self.max_linear, self.max_angular, self.stop_dist = max_linear, max_angular, stop_dist
        self.params = Params(speed=min(0.18, max_linear), max_ang=min(0.8, max_angular))
        root.title('Điều Khiển TurtleBot 4')
        root.resizable(False, False)
        root.configure(padx=14, pady=10)

        tk.Label(root, text='[ KHUNG GẦM TURTLEBOT 4 ]', font=('DejaVu Sans', 11, 'bold'), fg='#1f3fd0').pack(pady=(0, 6))
        self.lin_var = tk.DoubleVar(value=0.0)
        self.ang_var = tk.DoubleVar(value=0.0)
        self.lin_scale = self._slider('Tiến / Lùi  (m/s)      ◄ lùi  |  tiến ►  (kéo sang PHẢI để tiến)', self.lin_var, -max_linear, max_linear, 0.01)
        self.ang_scale = self._slider('Xoay tròn  (rad/s)      ◄ quay phải  |  quay trái ►', self.ang_var, -max_angular, max_angular, 0.01)

        tk.Label(root, text='[ CHẾ ĐỘ & AN TOÀN ]', font=('DejaVu Sans', 11, 'bold'), fg='#228b22').pack(pady=(10, 2))
        self.auto_var = tk.BooleanVar(value=False)
        self.safe_var = tk.BooleanVar(value=True)
        self.spring_var = tk.BooleanVar(value=False)
        tk.Checkbutton(root, text='Tự lái (bám tường bên phải, khảo sát vẽ map)', variable=self.auto_var,
                       command=self._on_auto, font=FONT).pack(anchor='w')
        tk.Checkbutton(root, text='Chống va chạm (chặn tiến khi vật cản < %.2f m)' % stop_dist,
                       variable=self.safe_var, font=FONT).pack(anchor='w')
        tk.Checkbutton(root, text='Tự về 0 khi thả thanh trượt', variable=self.spring_var, font=FONT).pack(anchor='w')

        if self.saver is not None:
            tk.Label(root, text='[ BẢN ĐỒ ]', font=('DejaVu Sans', 11, 'bold'), fg='#b8860b').pack(pady=(8, 2))
            self.autosave_var = tk.BooleanVar(value=True)
            self.saveclose_var = tk.BooleanVar(value=True)
            tk.Checkbutton(root, text='Tự lưu bản đồ mỗi %d giây' % int(autosave_period), variable=self.autosave_var,
                           font=FONT).pack(anchor='w')
            tk.Checkbutton(root, text='Tự lưu bản đồ khi đóng bảng này', variable=self.saveclose_var,
                           font=FONT).pack(anchor='w')
            tk.Button(root, text='LƯU BẢN ĐỒ NGAY', command=self.save_now, bg='#2e7d32', fg='white',
                      activebackground='#1b5e20', activeforeground='white', font=('DejaVu Sans', 10, 'bold'),
                      relief='flat', pady=4).pack(fill='x', pady=(4, 0))
            self.save_label = tk.Label(root, text='Bản đồ: chưa lưu', font=('DejaVu Sans', 9), anchor='w',
                                       justify='left', wraplength=380)
            self.save_label.pack(fill='x')
        self.status = tk.Label(root, text='', font=('DejaVu Sans Mono', 9), justify='left', anchor='w')
        self.status.pack(fill='x', pady=(8, 4))

        tk.Button(root, text='PHANH KHẨN CẤP XE  [SPACE]', command=self.emergency_stop, bg='#e5101d', fg='white',
                  activebackground='#b00d17', activeforeground='white', font=('DejaVu Sans', 11, 'bold'),
                  relief='flat', pady=8).pack(fill='x', pady=(6, 0))

        for scale in (self.lin_scale, self.ang_scale):
            scale.bind('<ButtonRelease-1>', self._on_release)
        root.bind('<space>', lambda e: self.emergency_stop())
        root.bind('<Up>', lambda e: self._nudge(self.lin_var, +0.05, self.max_linear))
        root.bind('<Down>', lambda e: self._nudge(self.lin_var, -0.05, self.max_linear))
        root.bind('<Left>', lambda e: self._nudge(self.ang_var, +0.1, self.max_angular))
        root.bind('<Right>', lambda e: self._nudge(self.ang_var, -0.1, self.max_angular))
        root.protocol('WM_DELETE_WINDOW', self.close)

        self.cmd = (0.0, 0.0)
        self.wall_ticks = 0
        self.root.after(100, self._tick)

    def _slider(self, title, var, lo, hi, step):
        tk.Label(self.root, text=title, font=FONT, anchor='w').pack(fill='x', pady=(8, 0))
        scale = tk.Scale(self.root, variable=var, from_=lo, to=hi, resolution=step, orient='horizontal',
                         length=380, showvalue=True, font=FONT, troughcolor='#c8c8c8', sliderlength=26)
        scale.pack(fill='x')
        return scale

    def _nudge(self, var, delta, limit):
        if self.auto_var.get():
            return
        var.set(max(-limit, min(limit, round(var.get() + delta, 3))))

    def _on_release(self, _event):
        if self.spring_var.get():
            self.lin_var.set(0.0)
            self.ang_var.set(0.0)

    def _on_auto(self):
        state = 'disabled' if self.auto_var.get() else 'normal'
        self.lin_scale.configure(state=state)
        self.ang_scale.configure(state=state)
        if not self.auto_var.get():
            self.emergency_stop()

    def emergency_stop(self):
        self.auto_var.set(False)
        self.lin_scale.configure(state='normal')
        self.ang_scale.configure(state='normal')
        self.lin_var.set(0.0)
        self.ang_var.set(0.0)
        self.cmd = (0.0, 0.0)
        for _ in range(3):
            self.bridge.publish(0.0, 0.0)

    def _tick(self):
        self.bridge.spin_once()
        front = self.bridge.front_distance()
        state = 'thu_cong'
        if self.auto_var.get():
            scan = self.bridge.scan
            if scan is None:
                lin, ang, state = 0.0, 0.0, 'cho_scan'
            else:
                lin, ang, state = compute_cmd(scan['ranges'], scan['angle_min'], scan['angle_inc'], self.params,
                                              scan['range_max'], self.wall_ticks > 0)
                self.wall_ticks = 60 if state == 'bam_tuong' else max(0, self.wall_ticks - 1)
            self.lin_var.set(round(lin, 3))
            self.ang_var.set(round(ang, 3))
        else:
            lin, ang = float(self.lin_var.get()), float(self.ang_var.get())
        if self.safe_var.get() and lin > 0 and front is not None and front < self.stop_dist:
            lin = 0.0
            state += ' / CHAN_VAT_CAN'
        self.cmd = (lin, ang)
        self.bridge.publish(lin, ang)
        self._autosave_tick()
        fr = 'n/a' if front is None else '%.2f m' % front
        self.status.configure(text='Lệnh gửi : v=%+.2f m/s  w=%+.2f rad/s\nVật cản trước : %s\nChế độ : %s' % (
            lin, ang, fr, state))
        self.root.after(100, self._tick)

    def save_now(self):
        if self.saver is None:
            return
        self.save_label.configure(text='Bản đồ: đang lưu...')
        self.saver.save_async(lambda msg: self.root.after(0, lambda: self.save_label.configure(text='Bản đồ: ' + msg)))

    def _autosave_tick(self):
        if self.saver is not None and self.autosave_var.get() and not self.saver.busy \
                and time.time() - self._last_autosave >= self.autosave_period:
            self._last_autosave = time.time()
            self.save_now()

    def close(self):
        for _ in range(3):
            self.bridge.publish(0.0, 0.0)
        if self.saver is not None and self.saveclose_var.get():
            self.save_label.configure(text='Bản đồ: đang lưu trước khi đóng...')
            self.root.update_idletasks()
            self.saver.save_blocking()
        self.root.destroy()


class RosBridge:
    """Dong goi ROS 2: publish /cmd_vel, doc /scan. Tach rieng de GUI kiem thu duoc khong can ROS."""

    def __init__(self, node, twist_cls, scan_cls, qos, cmd_topic, scan_topic):
        self.node, self.Twist = node, twist_cls
        self.pub = node.create_publisher(twist_cls, cmd_topic, 10)
        node.create_subscription(scan_cls, scan_topic, self._on_scan, qos)
        self.scan = None
        import rclpy
        self.rclpy = rclpy

    def _on_scan(self, msg):
        rmax = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0 else 8.0
        self.scan = {'ranges': list(msg.ranges), 'angle_min': msg.angle_min, 'angle_inc': msg.angle_increment,
                     'range_max': rmax}

    def spin_once(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish(self, lin, ang):
        msg = self.Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)

    def front_distance(self):
        if self.scan is None:
            return None
        from tb4_lab04.wall_follow import sector_min
        s = self.scan
        return sector_min(s['ranges'], s['angle_min'], s['angle_inc'], -20, 20, s['range_max'])


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('control_panel')
    node.declare_parameter('cmd_topic', '/cmd_vel')
    node.declare_parameter('scan_topic', '/scan')
    node.declare_parameter('max_linear', 0.3)
    node.declare_parameter('max_angular', 1.0)
    node.declare_parameter('stop_dist', 0.30)
    node.declare_parameter('map_dir', '~/tb4_ws/src/tb4_lab04/maps')
    node.declare_parameter('autosave_period', 120.0)
    g = node.get_parameter
    bridge = RosBridge(node, Twist, LaserScan, qos_profile_sensor_data, g('cmd_topic').value, g('scan_topic').value)
    root = tk.Tk()
    Panel(root, bridge, g('max_linear').value, g('max_angular').value, g('stop_dist').value,
          saver=MapSaver(g('map_dir').value), autosave_period=g('autosave_period').value)
    try:
        root.mainloop()
    finally:
        bridge.publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
