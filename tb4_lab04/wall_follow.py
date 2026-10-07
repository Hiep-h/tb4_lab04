"""Thuat toan bam tuong ben phai (thuan tuy, khong phu thuoc ROS) de de kiem thu.

Quy uoc goc: 0 = huong truoc robot, duong = quay trai (nguoc chieu kim dong ho), am = ben phai.
"""
import math
from dataclasses import dataclass


@dataclass
class Params:
    speed: float = 0.18          # van toc tien toi da (m/s)
    max_ang: float = 0.8         # van toc quay toi da (rad/s)
    target_dist: float = 0.55    # khoang cach muon giu voi tuong ben phai (m)
    front_stop: float = 0.32     # gan hon thi dung tien va quay trai (m)
    front_slow: float = 0.65     # gan hon thi giam toc va quay trai (m)
    wall_lost: float = 1.3       # xa hon thi coi nhu mat tuong, di vong sang phai (m)
    kp: float = 1.6              # he so bam tuong


def sector_min(ranges, angle_min, angle_inc, lo_deg, hi_deg, rmax):
    """Khoang cach nho nhat trong cung [lo_deg, hi_deg]; gia tri khong hop le bi bo qua."""
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    best = rmax
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        a = math.atan2(math.sin(a), math.cos(a))
        if lo <= a <= hi and r is not None and math.isfinite(r) and r > 0.05:
            best = min(best, r)
    return best


def compute_cmd(ranges, angle_min, angle_inc, p: Params, rmax: float = 8.0):
    """Tra ve (linear, angular, trang_thai)."""
    front = sector_min(ranges, angle_min, angle_inc, -25, 25, rmax)
    right = sector_min(ranges, angle_min, angle_inc, -100, -70, rmax)
    front_right = sector_min(ranges, angle_min, angle_inc, -65, -25, rmax)

    if front < p.front_stop:
        return 0.0, p.max_ang, 'quay_trai'
    if front < p.front_slow:
        return 0.4 * p.speed, 0.7 * p.max_ang, 'ne_vat_can'

    # khoang cach vuong goc uoc luong tu huong cheo phia truoc ben phai (45 do)
    d = min(right, front_right * math.cos(math.radians(45)))
    if d > p.wall_lost:
        return 0.8 * p.speed, -0.6 * p.max_ang, 'tim_tuong'
    ang = -p.kp * (d - p.target_dist)
    ang = max(-p.max_ang, min(p.max_ang, ang))
    lin = p.speed * (1.0 - 0.5 * abs(ang) / p.max_ang)
    return lin, ang, 'bam_tuong'
