#!/usr/bin/env python3
import os
import re
import sys

import numpy as np
from PIL import Image

UNK, OCC = 205, 0


def footprint(a):
    occ = a == OCC
    H, W = a.shape
    cols = np.where(occ[H // 2])[0]
    left, right = int(cols.min()), int(cols.max())
    rows = occ[:, left:right + 1].sum(1)
    top = 30 + int(np.argmax(rows[30:H // 2]))

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
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + '_clean.pgm'
    img = np.array(Image.open(src))
    out = clean(img)
    Image.fromarray(out).save(dst)
    yaml_path = os.path.splitext(dst)[0] + '.yaml'
    if os.path.exists(yaml_path):
        txt = open(yaml_path).read()
        txt = re.sub(r'free_thresh:\s*[0-9.]+', 'free_thresh: 0.196', txt)
        open(yaml_path, 'w').write(txt)
    print('chua biet %d -> %d, vat can %d -> %d' % ((img == UNK).sum(), (out == UNK).sum(), (img == OCC).sum(), (out == OCC).sum()))
