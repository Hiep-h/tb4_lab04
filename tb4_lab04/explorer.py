"""Bo nao kham pha tu dong (thuan Python, khong phu thuoc ROS): nhan ban do + vi tri, tra ve lenh van toc.

Trang thai: 'spin' (xoay tai cho 1 vong de co ban do ban dau) -> 'explore' (di toi bien gan nhat) -> 'done'.
"""
import math

import numpy as np

from tb4_lab04 import frontier as F


class Explorer:
    def __init__(self, vmax=0.3, wmax=1.0, lookahead=0.5, factor=2, inflate_m=0.24, clearance_m=0.25,
                 replan_period=1.5, spin_time=7.5, min_cluster=3, stuck_time=8.0, done_after=3, front_stop=0.28):
        self.vmax, self.wmax, self.lookahead, self.factor = vmax, wmax, lookahead, factor
        self.inflate_m, self.clearance_m = inflate_m, clearance_m
        self.replan_period, self.spin_time = replan_period, spin_time
        self.min_cluster, self.stuck_time, self.done_after = min_cluster, stuck_time, done_after
        self.front_stop = front_stop
        self.state = 'spin'
        self.t_start = None
        self.path = []            # danh sach (x, y) the gioi
        self.goal_xy = None
        self.last_plan_t = -1e9
        self.no_goal_count = 0
        self.blacklist = []       # (row, col) o luoi giam do phan giai
        self.progress_t = None
        self.progress_xy = None
        self.recover_until = -1.0
        self.recover_dir = 1.0
        self.last_info = ''
        self.goal_cells = None

    # ---- lap ke hoach ----
    def _plan(self, grid, res, origin, pose):
        f = self.factor
        g = F.downsample(grid, f)
        r2 = res * f
        ox, oy = origin
        x, y, _ = pose
        sr = int((y - oy) / r2)
        sc = int((x - ox) / r2)
        h, w = g.shape
        if not (0 <= sr < h and 0 <= sc < w):
            return None
        free, pen = F.traversable(g, self.inflate_m / r2, self.clearance_m / r2)
        # cho phep robot di ra khoi vung phong rong xung quanh no
        rr0, rr1, cc0, cc1 = max(0, sr - 2), min(h, sr + 3), max(0, sc - 2), min(w, sc + 3)
        blk = g[rr0:rr1, cc0:cc1]
        free[rr0:rr1, cc0:cc1] |= (blk == F.FREE)
        dist, parent = F.dijkstra(free, pen, (sr, sc))
        fm = F.frontier_mask(g) & free
        cl = F.clusters(fm, self.min_cluster)
        best = F.choose_goal(dist, cl, r2, self.blacklist)
        if best is None:
            return None
        cells = F.extract_path(parent, (best[0], best[1]), w)
        pts = [(ox + (c + 0.5) * r2, oy + (r + 0.5) * r2) for r, c in cells]
        self.goal_cells = (best[0], best[1])
        return F.smooth(pts, 0.15), len(cl)

    # ---- vong lap dieu khien ----
    def update(self, t, grid, res, origin, pose, front_dist=None):
        """Tra ve (v, w, state). grid: mang (h, w) int; pose = (x, y, yaw) trong khung map."""
        if self.t_start is None:
            self.t_start = t
            self.progress_t, self.progress_xy = t, (pose[0], pose[1])
        if self.state == 'done':
            return 0.0, 0.0, 'done'
        if self.state == 'spin':
            if t - self.t_start < self.spin_time:
                return 0.0, 0.8, 'spin'
            self.state = 'explore'
            self.progress_t, self.progress_xy = t, (pose[0], pose[1])
        # phuc hoi sau khi ket: quay tai cho
        if t < self.recover_until:
            return 0.0, 0.8 * self.recover_dir, 'phuc_hoi'
        # an toan phan xa: vat can sat truoc mat -> quay
        if front_dist is not None and front_dist < self.front_stop:
            self.last_plan_t = -1e9
            return 0.0, 0.8, 'ne_vat_can'

        reached = bool(self.path) and math.hypot(self.path[-1][0] - pose[0], self.path[-1][1] - pose[1]) < 0.3
        if reached or not self.path or t - self.last_plan_t >= self.replan_period:
            self.last_plan_t = t
            res_plan = self._plan(grid, res, origin, pose)
            if res_plan is None and self.blacklist:
                self.blacklist = []                      # thu lai khong co danh sach den truoc khi ket luan het bien
                res_plan = self._plan(grid, res, origin, pose)
            if res_plan is None:
                self.no_goal_count += 1
                self.path = []
                if self.no_goal_count >= self.done_after:
                    self.state = 'done'
                    return 0.0, 0.0, 'done'
                return 0.0, 0.0, 'cho_ban_do'
            self.no_goal_count = 0
            self.path = res_plan[0]
            self.goal_xy = self.path[-1]

        # phat hien ket: it di chuyen trong stuck_time giay
        if math.hypot(pose[0] - self.progress_xy[0], pose[1] - self.progress_xy[1]) > 0.2:
            self.progress_t, self.progress_xy = t, (pose[0], pose[1])
        elif t - self.progress_t > self.stuck_time:
            if self.goal_cells is not None:
                self.blacklist.append(self.goal_cells)
            self.recover_until = t + 3.0
            self.recover_dir = -self.recover_dir
            self.progress_t, self.progress_xy = t, (pose[0], pose[1])
            self.path = []
            return 0.0, 0.8 * self.recover_dir, 'ket_quay'

        v, w, remaining = F.pure_pursuit(self.path, pose, self.lookahead, self.vmax, self.wmax)
        return v, w, 'kham_pha'
