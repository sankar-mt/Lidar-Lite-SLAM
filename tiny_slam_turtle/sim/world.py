import pybullet as p
import pybullet_data
import numpy as np
import math

class SimpleWorld:
    def __init__(self, gui=False, seed=0):
        self.gui = gui
        self.seed = seed
        if gui:
            self.cid = p.connect(p.GUI)
        else:
            self.cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")
        self._build_room()
        self.max_range = 5.0

    def _build_room(self):
        wall_th, size, wall_height = 0.1, 6.0, 0.5
        half = size/2.0
        def make_wall(x, y, length, thickness, yaw=0.0):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length/2, thickness/2, wall_height/2])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[length/2, thickness/2, wall_height/2], rgbaColor=[0.8, 0.8, 0.8, 1])
            quat = p.getQuaternionFromEuler([0, 0, yaw])
            return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                     basePosition=[x, y, wall_height/2], baseOrientation=quat)
        make_wall(0,  half, size, wall_th, 0)
        make_wall(0, -half, size, wall_th, 0)
        make_wall( half, 0, size, wall_th, math.pi/2)
        make_wall(-half, 0, size, wall_th, math.pi/2)
        np.random.seed(42)
        for _ in range(6):
            x = np.random.uniform(-2.0, 2.0); y = np.random.uniform(-2.0, 2.0)
            l = np.random.uniform(0.5, 1.2); w = np.random.uniform(0.1, 0.2)
            yaw = np.random.uniform(0, math.pi)
            make_wall(x, y, l, w, yaw)

    def step(self): p.stepSimulation()
    def disconnect(self): p.disconnect()

    def ray_scan(self, pose, angles):
        x, y, th = pose
        starts, ends = [], []
        for a in angles:
            ang = th + a
            sx, sy = x, y
            ex = x + self.max_range * math.cos(ang)
            ey = y + self.max_range * math.sin(ang)
            sz = ez = 0.1
            starts.append([sx, sy, sz]); ends.append([ex, ey, ez])
        res = p.rayTestBatch(starts, ends)
        dists = []
        for r in res:
            hit = r[2]
            dists.append(hit * self.max_range if hit < 1.0 else self.max_range)
        return np.array(dists, dtype=np.float32)
