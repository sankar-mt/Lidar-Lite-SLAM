import numpy as np
import math

def raycast_grid(occ, pose, angles, max_range):
    x, y, th = pose
    res = []
    step = occ.res * 0.5
    occ_prob = occ.prob()
    for a in angles:
        ang = th + a; d = 0.0
        while d < max_range:
            px = x + d * math.cos(ang); py = y + d * math.sin(ang)
            mx, my = occ.world_to_map(px, py)
            if not occ.in_bounds(mx, my):
                d = max_range; break
            if occ_prob[my, mx] > 0.65: break
            d += step
        res.append(min(d, max_range))
    return np.array(res, dtype=np.float32)
