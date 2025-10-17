import math

class DiffDriveRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, wheel_base=0.3, max_speed=0.6):
        self.x, self.y, self.theta = x, y, theta
        self.wheel_base, self.max_speed = wheel_base, max_speed
        self.v_l = self.v_r = 0.0

    def set_velocities(self, v_l, v_r):
        self.v_l = max(-self.max_speed, min(self.max_speed, float(v_l)))
        self.v_r = max(-self.max_speed, min(self.max_speed, float(v_r)))

    def step(self, dt):
        vl, vr = self.v_l, self.v_r
        v = (vr + vl) / 2.0
        omega = (vr - vl) / self.wheel_base
        if abs(omega) < 1e-6:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            R = v / omega
            dtheta = omega * dt
            cx = self.x - R * math.sin(self.theta)
            cy = self.y + R * math.cos(self.theta)
            self.theta = (self.theta + dtheta + math.pi) % (2*math.pi) - math.pi
            self.x = cx + R * math.sin(self.theta)
            self.y = cy - R * math.cos(self.theta)

    def pose(self): return (self.x, self.y, self.theta)
