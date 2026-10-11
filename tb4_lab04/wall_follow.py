import math
from dataclasses import dataclass


@dataclass
class Params:
    speed: float = 0.18
    max_ang: float = 0.8
    target_dist: float = 0.55
    front_stop: float = 0.32
    front_slow: float = 0.65
    wall_lost: float = 1.3
    kp: float = 1.6


def sector_min(ranges, angle_min, angle_inc, lo_deg, hi_deg, rmax):
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    best = rmax
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        a = math.atan2(math.sin(a), math.cos(a))
        if lo <= a <= hi and r is not None and math.isfinite(r) and r > 0.05:
            best = min(best, r)
    return best


def compute_cmd(ranges, angle_min, angle_inc, p: Params, rmax: float = 8.0, wall_recent: bool = True):
    front = sector_min(ranges, angle_min, angle_inc, -25, 25, rmax)
    right = sector_min(ranges, angle_min, angle_inc, -100, -70, rmax)
    front_right = sector_min(ranges, angle_min, angle_inc, -65, -25, rmax)

    if front < p.front_stop:
        return 0.0, p.max_ang, 'quay_trai'
    if front < p.front_slow:
        return 0.4 * p.speed, 0.7 * p.max_ang, 'ne_vat_can'

    if right > p.wall_lost:

        if front_right < 0.9:

            return 0.5 * p.speed, 0.5 * p.max_ang, 'ne_vat_can'
        if wall_recent:
            return 0.8 * p.speed, -0.6 * p.max_ang, 'cua_goc'
        return p.speed, 0.0, 'tim_tuong'


    d = min(right, front_right * math.cos(math.radians(45)))
    ang = -p.kp * (d - p.target_dist)
    ang = max(-p.max_ang, min(p.max_ang, ang))
    lin = p.speed * (1.0 - 0.5 * abs(ang) / p.max_ang)
    return lin, ang, 'bam_tuong'
