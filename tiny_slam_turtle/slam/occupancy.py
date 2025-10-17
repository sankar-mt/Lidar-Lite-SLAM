import numpy as np
import math

class OccupancyGrid:
    def __init__(self, size_m=8.0, resolution=0.05, lo_occ=0.6, lo_free=-0.1, lo_min=-3.0, lo_max=3.0):
        self.size_m = float(size_m); self.res = float(resolution)
        self.n = int(self.size_m / self.res)
        self.grid = np.zeros((self.n, self.n), dtype=np.float32)
        self.lo_occ, self.lo_free = float(lo_occ), float(lo_free)
        self.lo_min, self.lo_max = float(lo_min), float(lo_max)

    def world_to_map(self, x, y):
        half = self.size_m / 2.0
        mx = int((x + half) / self.res); my = int((y + half) / self.res)
        return mx, my

    def map_to_world(self, mx, my):
        half = self.size_m / 2.0
        x = mx * self.res - half + 0.5 * self.res
        y = my * self.res - half + 0.5 * self.res
        return x, y

    def in_bounds(self, mx, my): return 0 <= mx < self.n and 0 <= my < self.n
    def clamp(self): np.clip(self.grid, self.lo_min, self.lo_max, out=self.grid)

    def bresenham(self, x0, y0, x1, y1):
        dx = abs(x1 - x0); dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
        err = dx + dy; x, y = x0, y0; cells = []
        while True:
            cells.append((x, y))
            if x == x1 and y == y1: break
            e2 = 2 * err
            if e2 >= dy: err += dy; x += sx
            if e2 <= dx: err += dx; y += sy
        return cells

    def update_from_scan(self, pose, ranges, angles, max_range):
        x, y, th = pose
        rmx, rmy = self.world_to_map(x, y)
        for d, a in zip(ranges, angles):
            ang = th + a
            ex = x + d * math.cos(ang); ey = y + d * math.sin(ang)
            emx, emy = self.world_to_map(ex, ey)
            cells = self.bresenham(rmx, rmy, emx, emy)
            for (mx, my) in cells[:-1]:
                if self.in_bounds(mx, my): self.grid[my, mx] += self.lo_free
            if d < max_range * 0.999:
                if self.in_bounds(emx, emy): self.grid[emy, emx] += self.lo_occ
        self.clamp()

    def prob(self): return 1.0 - 1.0 / (1.0 + np.exp(self.grid))
