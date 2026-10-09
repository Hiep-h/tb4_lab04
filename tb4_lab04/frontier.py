"""Thuat toan kham pha theo bien (frontier) - thuan tuy numpy, khong phu thuoc ROS de kiem thu duoc.

Y tuong: tren ban do chiem dung (OccupancyGrid), "bien" la o trong (free) nam sat o chua biet (unknown).
Robot tu chon bien o gan nhat theo DUONG DI THAT (Dijkstra tren cac o trong da phong rong quanh vat can),
di toi do, ban do lon them, lap lai cho den khi khong con bien nao toi duoc.

Quy uoc luoi: mang numpy shape (h, w); gia tri -1 chua biet, 0 trong, 100 vat can. Toa do o (row, col);
the gioi: x = ox + (col + 0.5) * res, y = oy + (row + 0.5) * res.
"""
import heapq
import math

import numpy as np

UNKNOWN, FREE, OCC = -1, 0, 100


def downsample(grid, factor):
    """Giam do phan giai (bao thu): co 1 o vat can trong khoi -> vat can; neu khong, trong khi so o trong >= so o chua biet.
    Cung chuan hoa gia tri: <0 chua biet, 0..64 trong, >=65 vat can."""
    grid = np.asarray(grid)
    if factor <= 1:
        out = np.full(grid.shape, UNKNOWN, dtype=np.int16)
        out[(grid >= 0) & (grid < 65)] = FREE
        out[grid >= 65] = OCC
        return out
    h, w = grid.shape
    h2, w2 = h // factor, w // factor
    g = grid[:h2 * factor, :w2 * factor].reshape(h2, factor, w2, factor).transpose(0, 2, 1, 3).reshape(h2, w2, -1)
    occ = (g >= 65).any(axis=2)
    free = ((g >= 0) & (g < 65)).sum(axis=2)
    unk = (g < 0).sum(axis=2)
    out = np.full((h2, w2), UNKNOWN, dtype=np.int16)
    out[(free > 0) & (free >= unk)] = FREE
    out[occ] = OCC
    return out


def _shift(a, dr, dc):
    """Tra ve b sao cho b[r, c] = a[r - dr, c - dc] (o ngoai bien = False)."""
    h, w = a.shape
    b = np.zeros_like(a)
    rs, re = max(0, dr), h + min(0, dr)
    cs, ce = max(0, dc), w + min(0, dc)
    if re > rs and ce > cs:
        b[rs:re, cs:ce] = a[rs - dr:re - dr, cs - dc:ce - dc]
    return b


def dilate(mask, radius_cells):
    """Phong rong mat na boolean theo hinh dia."""
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    r = int(math.ceil(radius_cells))
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if (dr or dc) and dr * dr + dc * dc <= radius_cells * radius_cells:
                out |= _shift(mask, dr, dc)
    return out


def frontier_mask(grid):
    """O trong co it nhat 1 o chua biet trong 8 o lang gieng."""
    unk = grid == UNKNOWN
    near_unk = np.zeros_like(unk)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr or dc:
                near_unk |= _shift(unk, dr, dc)
    return (grid == FREE) & near_unk


def traversable(grid, inflate_cells, clearance_cells=0):
    """Mat na o di duoc (trong, khong gan vat can) va ban do chi phi phat khi gan vat can."""
    occ = grid >= 65
    blocked = dilate(occ, inflate_cells)
    # o chua biet khong di duoc (an toan); bien van toi duoc vi no la o trong
    free = (grid == FREE) & ~blocked
    penalty = np.zeros(grid.shape, dtype=np.float32)
    if clearance_cells > 0:
        near = dilate(occ, inflate_cells + clearance_cells) & ~blocked
        penalty[near] = 2.0
    return free, penalty


def dijkstra(free, penalty, start):
    """Dijkstra 8 huong tu o start (row, col) tren cac o free. Tra ve (dist, parent) dang mang phang."""
    h, w = free.shape
    n = h * w
    dist = np.full(n, np.inf, dtype=np.float64)
    parent = np.full(n, -1, dtype=np.int64)
    sr, sc = start
    if not (0 <= sr < h and 0 <= sc < w):
        return dist.reshape(h, w), parent.reshape(h, w)
    # neu o xuat phat khong "free" (vi phong rong), van cho di ra
    s = sr * w + sc
    dist[s] = 0.0
    freef = free.ravel()
    pen = penalty.ravel()
    pq = [(0.0, s)]
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        ur, uc = divmod(u, w)
        for dr, dc, c in nbrs:
            vr, vc = ur + dr, uc + dc
            if vr < 0 or vr >= h or vc < 0 or vc >= w:
                continue
            v = vr * w + vc
            if not freef[v]:
                continue
            if dr and dc and not (freef[ur * w + vc] and freef[vr * w + uc]):
                continue  # khong cat goc vat can
            nd = d + c + pen[v]
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd, v))
    return dist.reshape(h, w), parent.reshape(h, w)


def clusters(mask, min_size=3):
    """Gom cac o bien lien nhau (8 huong). Tra ve danh sach mang toa do (k,2)."""
    h, w = mask.shape
    seen = np.zeros_like(mask)
    out = []
    pts = np.argwhere(mask)
    for r0, c0 in pts:
        if seen[r0, c0]:
            continue
        stack = [(r0, c0)]
        seen[r0, c0] = True
        cells = []
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        if len(cells) >= min_size:
            out.append(np.array(cells))
    return out


def choose_goal(dist, frontiers, res, blacklist=(), size_weight=0.4, min_goal_dist=0.35):
    """Chon bien co diem so tot nhat: duong di ngan, bien lon (nhieu thong tin). Tra ve (row, col, score) hoac None."""
    best = None
    for cells in frontiers:
        d = dist[cells[:, 0], cells[:, 1]]
        ok = np.isfinite(d)
        if not ok.any():
            continue
        # o gan nhat trong cum (theo duong di)
        i = int(np.argmin(np.where(ok, d, np.inf)))
        r, c = int(cells[i, 0]), int(cells[i, 1])
        dm = float(d[i]) * res
        if dm < min_goal_dist:
            continue
        if any(math.hypot(r - br, c - bc) * res < 0.5 for br, bc in blacklist):
            continue
        score = dm - size_weight * math.sqrt(len(cells)) * res * 10.0 * 0.1
        if best is None or score < best[2]:
            best = (r, c, score)
    return best


def extract_path(parent, goal, w):
    """Chuoi o (row, col) tu xuat phat den goal."""
    path = []
    u = goal[0] * w + goal[1]
    guard = 0
    while u != -1 and guard < parent.size:
        path.append(divmod(u, w))
        u = int(parent.ravel()[u])
        guard += 1
    path.reverse()
    return path


def smooth(path_xy, step=0.15):
    """Lay lai diem doc duong di moi `step` met (de pure pursuit muot)."""
    if len(path_xy) < 2:
        return list(path_xy)
    out = [path_xy[0]]
    acc = 0.0
    for a, b in zip(path_xy[:-1], path_xy[1:]):
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        if acc >= step:
            out.append(b)
            acc = 0.0
    if out[-1] != path_xy[-1]:
        out.append(path_xy[-1])
    return out


def pure_pursuit(path_xy, pose, lookahead=0.5, vmax=0.3, wmax=1.0):
    """Lenh (v, w) bam duong di. pose = (x, y, yaw). Tra ve (v, w, con_lai) voi con_lai = khoang cach toi cuoi duong."""
    x, y, yaw = pose
    if not path_xy:
        return 0.0, 0.0, 0.0
    # diem gan nhat roi tien len den diem cach lookahead
    dmin, imin = 1e9, 0
    for i, (px, py) in enumerate(path_xy):
        d = math.hypot(px - x, py - y)
        if d < dmin:
            dmin, imin = d, i
    tgt = path_xy[-1]
    for px, py in path_xy[imin:]:
        if math.hypot(px - x, py - y) >= lookahead:
            tgt = (px, py)
            break
    remaining = math.hypot(path_xy[-1][0] - x, path_xy[-1][1] - y)
    err = math.atan2(tgt[1] - y, tgt[0] - x) - yaw
    err = math.atan2(math.sin(err), math.cos(err))
    if abs(err) > math.radians(55):
        return 0.0, max(-wmax, min(wmax, 1.8 * err)), remaining       # quay tai cho truoc
    w = max(-wmax, min(wmax, 2.0 * err))
    v = vmax * max(0.25, 1.0 - abs(err) / math.radians(55))
    v = min(v, max(0.08, remaining))                                     # giam toc khi toi dich
    return v, w, remaining
