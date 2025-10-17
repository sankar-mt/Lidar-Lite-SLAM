import numpy as np
import math
from .raycast import raycast_grid

class ParticleFilter:
    def __init__(self, occ, n_particles=300, init_pose=(0,0,0), trans_noise=0.02, rot_noise=0.01):
        self.occ = occ; self.n = int(n_particles)
        self.p = np.zeros((self.n, 3), dtype=np.float32)
        self.w = np.ones(self.n, dtype=np.float32) / self.n
        self.trans_noise = float(trans_noise); self.rot_noise = float(rot_noise)
        self.reset(init_pose)

    def reset(self, init_pose):
        x0, y0, th0 = init_pose
        self.p[:, 0] = x0 + np.random.normal(0, 0.1, self.n)
        self.p[:, 1] = y0 + np.random.normal(0, 0.1, self.n)
        self.p[:, 2] = th0 + np.random.normal(0, 0.05, self.n)
        self.w[:] = 1.0 / self.n

    def predict(self, v, omega, dt):
        n = self.n
        v_s = v + np.random.normal(0, self.trans_noise, n)
        o_s = omega + np.random.normal(0, self.rot_noise, n)
        th = self.p[:, 2]
        self.p[:, 0] += v_s * np.cos(th) * dt
        self.p[:, 1] += v_s * np.sin(th) * dt
        self.p[:, 2] = ((th + o_s * dt + math.pi) % (2*math.pi)) - math.pi

    def update(self, z_obs, angles, max_range, sigma=0.08):
        eps = 1e-9; var = sigma**2
        new_w = np.zeros_like(self.w)
        for i in range(self.n):
            z_hat = raycast_grid(self.occ, tuple(self.p[i]), angles, max_range)
            err = z_obs - z_hat
            ll = -0.5 * np.sum((err*err) / var)
            new_w[i] = math.exp(ll)
        s = np.sum(new_w) + eps
        self.w = new_w / s

    def resample(self):
        # Robust low-variance/systematic resampling using searchsorted.
        n = self.n
        w = self.w + 1e-12
        w = w / w.sum()
        cumsum = np.cumsum(w)
        cumsum[-1] = 1.0  # ensure last bin closes at 1.0
        positions = (np.arange(n) + np.random.uniform()) / n
        indexes = np.searchsorted(cumsum, positions, side='left')
        self.p[:] = self.p[indexes]
        self.w[:] = 1.0 / n

    def estimate(self):
        x = np.average(self.p[:,0], weights=self.w)
        y = np.average(self.p[:,1], weights=self.w)
        c = np.average(np.cos(self.p[:,2]), weights=self.w)
        s = np.average(np.sin(self.p[:,2]), weights=self.w)
        th = math.atan2(s, c)
        return (float(x), float(y), float(th))
