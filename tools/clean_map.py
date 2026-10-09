#!/usr/bin/env python3
"""Lam sach map.pgm cua world small_house: moi o 'chua biet' (205) NAM TRONG nha -> vat can (0).

Ly do: than do noi that (giuong, sofa, tu bep, tu quan ao...) la khoi dac ma LiDAR khong nhin xuyen qua duoc, nen ben trong
con xam ('chua biet'). Cac o do khong phai vung chua kham pha. Vung chua biet NGOAI nha (ngoai tuong) giu nguyen.

Chu vi nha doc tu chinh ban do: than nha = hinh chu nhat, ban cong = hinh chu nhat nho nhat len phia tren (cot tuong ban cong
tim bang cot co nhieu o den nhat o nua tren). Dung: python3 tools/clean_map.py maps/map.pgm [map_sach.pgm]
Neu co map.yaml canh ban do, free_thresh duoc dat 0.196 de Nav2 hieu 205 la 'chua biet' (mac dinh 0.25 se coi la trong).
"""
import os
import re
import sys

import numpy as np
from PIL import Image

UNK, OCC = 205, 0


def footprint(a):
    """Tra ve mat na True cho cac o nam trong chu vi nha."""
    occ = a == OCC
    H, W = a.shape
    cols = np.where(occ[H // 2])[0]                       # hang giua: tuong trai va tuong phai
    left, right = int(cols.min()), int(cols.max())
    rows = occ[:, left:right + 1].sum(1)                  # hang co nhieu o den nhat o 1/4 tren/duoi = tuong ngang
    top = 30 + int(np.argmax(rows[30:H // 2]))
    # ban cong: hai cot den cao nhat o phan tren tuong tren
    upper = occ[:max(top - 1, 1)].sum(0)
    bc = sorted(int(c) for c in np.argsort(-upper)[:2])
    mask = np.zeros((H, W), bool)
    mask[top:, left:right + 1] = True
    if bc[1] - bc[0] > 20:
        mask[:top, bc[0]:bc[1] + 1] = True
    return mask


def clean(a):
    out = a.copy()
    out[(a == UNK) & footprint(a)] = OCC
    return out


if __name__ == '__main__':
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src
    img = np.array(Image.open(src))
    out = clean(img)
    Image.fromarray(out).save(dst)
    yaml_path = os.path.splitext(dst)[0] + '.yaml'
    if os.path.exists(yaml_path):
        txt = open(yaml_path).read()
        txt = re.sub(r'free_thresh:\s*[0-9.]+', 'free_thresh: 0.196', txt)
        open(yaml_path, 'w').write(txt)
    print('chua biet %d -> %d, vat can %d -> %d' % ((img == UNK).sum(), (out == UNK).sum(), (img == OCC).sum(), (out == OCC).sum()))
