import time, math
import numpy as np
import matplotlib.pyplot as plt

from sim.world import SimpleWorld
from sim.robot import DiffDriveRobot
from slam.occupancy import OccupancyGrid
from slam.mcl import ParticleFilter

def main():
    DT = 0.1
    angles = np.linspace(-math.pi/2, math.pi/2, 121)
    world = SimpleWorld(gui=True, seed=0)
    robot = DiffDriveRobot(x=-2.0, y=-2.0, theta=0.0)
    occ = OccupancyGrid(size_m=8.0, resolution=0.05)
    pf = ParticleFilter(occ, n_particles=400, init_pose=robot.pose(), trans_noise=0.03, rot_noise=0.02)

    plt.ion()
    fig, ax = plt.subplots()
    im = ax.imshow(occ.prob(), origin='lower', cmap='gray', vmin=0.0, vmax=1.0)
    scat_parts = ax.scatter([], [], s=2, c='red')
    scat_robot = ax.scatter([], [], marker='x', c='blue')
    ax.set_title('Tiny-SLAM Turtle: Occupancy Map + Particles')
    fig.tight_layout()

    t = 0.0; last_vis = 0.0
    goal = np.array([2.0, 2.0])
    while t < 60.0:
        x, y, th = robot.pose()
        to_goal = goal - np.array([x, y])
        if np.linalg.norm(to_goal) < 0.2:
            goal = np.random.uniform(-2.5, 2.5, size=(2,))
        desired_th = math.atan2(to_goal[1], to_goal[0])
        e_th = ((desired_th - th + math.pi) % (2*math.pi)) - math.pi
        v = 0.25 * np.clip(np.linalg.norm(to_goal), 0.0, 1.0)
        omega = 1.5 * e_th

        L = robot.wheel_base
        v_r = v + 0.5 * L * omega; v_l = v - 0.5 * L * omega
        robot.set_velocities(v_l, v_r)
        robot.step(DT); world.step()

        z = world.ray_scan(robot.pose(), angles)
        z = z + np.random.normal(0, 0.01, size=z.shape)
        z = np.clip(z, 0.0, world.max_range)

        occ.update_from_scan(robot.pose(), z, angles, world.max_range)

        pf.predict(v=v, omega=omega, dt=DT)
        pf.update(z_obs=z, angles=angles, max_range=world.max_range, sigma=0.08)
        pf.resample()
        est = pf.estimate()

        if t - last_vis > 0.1:
            prob = occ.prob(); im.set_data(prob)
            px, py = pf.p[:,0], pf.p[:,1]
            pmx, pmy = [], []
            for xx, yy in zip(px, py):
                mx, my = occ.world_to_map(xx, yy)
                pmx.append(mx); pmy.append(my)
            scat_parts.set_offsets(np.c_[pmx, pmy])
            rx, ry = occ.world_to_map(x, y)
            scat_robot.set_offsets([[rx, ry]])
            plt.pause(0.001); last_vis = t

        t += DT

    print("Done. Close the plot to exit.")
    plt.ioff(); plt.show()
    world.disconnect()

if __name__ == "__main__":
    main()
