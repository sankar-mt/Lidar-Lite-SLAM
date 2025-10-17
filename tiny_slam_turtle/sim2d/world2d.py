import numpy as np
import math

class SimpleWorld2D:
    """Pure-NumPy 2D world with line-segment obstacles and ray casting.
    Units: meters. z is ignored. No external deps beyond numpy.
    """
    def __init__(self, seed=0, size=6.0, max_range=5.0):
        self.rng = np.random.default_rng(seed)
        self.size = float(size)
        self.half = self.size/2.0
        self.max_range = float(max_range)
        # Build perimeter and some random segments
        self.segments = []
        # Perimeter (square room)
        s = self.half
        self._add_seg((-s, -s), ( s, -s))
        self._add_seg(( s, -s), ( s,  s))
        self._add_seg(( s,  s), (-s,  s))
        self._add_seg((-s,  s), (-s, -s))
        # Internal obstacles
        for _ in range(6):
            x = self.rng.uniform(-2.0, 2.0)
            y = self.rng.uniform(-2.0, 2.0)
            l = self.rng.uniform(0.5, 1.2)
            yaw = self.rng.uniform(0, math.pi)
            dx = 0.5*l*math.cos(yaw)
            dy = 0.5*l*math.sin(yaw)
            self._add_seg((x-dx, y-dy), (x+dx, y+dy))

    def _add_seg(self, a, b):
        self.segments.append((np.array(a, dtype=float), np.array(b, dtype=float)))

    def step(self):
        pass  # no physics

    def disconnect(self):
        pass

    def ray_scan(self, pose, angles):
        x, y, th = pose
        origin = np.array([x, y], dtype=float)
        dists = []
        for a in angles:
            ang = th + a
            dir = np.array([math.cos(ang), math.sin(ang)], dtype=float)
            d = self._cast_ray(origin, dir, self.max_range)
            dists.append(d)
        return np.array(dists, dtype=np.float32)

    def _cast_ray(self, o, d, max_r):
        # Ray: o + t d, t>=0. Segment: p + u (q-p), u in [0,1]
        best = max_r
        for p, q in self.segments:
            v = q - p
            denom = d[0]*(-v[1]) + d[1]*(v[0])
            if abs(denom) < 1e-9:
                continue
            rel = p - o
            t = (rel[0]*(-v[1]) + rel[1]*(v[0])) / denom
            if t < 0: 
                continue
            u = (rel[0]*d[1] - rel[1]*d[0]) / denom
            if 0.0 <= u <= 1.0:
                dist = t
                if 0.0 <= dist < best:
                    best = dist
        return min(best, max_r)
