import argparse, math, time, random
import numpy as np
import matplotlib.pyplot as plt

# Try PyBullet; fall back to pure 2D sim if unavailable
try:
    from sim.world import SimpleWorld as _World3D
    HAVE_PYBULLET = True
except Exception:
    HAVE_PYBULLET = False
from sim2d.world2d import SimpleWorld2D

from sim.robot import DiffDriveRobot
from slam.occupancy import OccupancyGrid
from slam.mcl import ParticleFilter
from planning.astar import astar
from utils.run_logger import RunLogger

def choose_random_free(binary):
    H, W = binary.shape
    for _ in range(10000):
        y = np.random.randint(0, H)
        x = np.random.randint(0, W)
        if binary[y, x] == 0:
            return (y, x)
    return (H//2, W//2)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0, help='Run duration')
    ap.add_argument('--record', action='store_true', help='Save frames and try to make MP4')
    ap.add_argument('--save', action='store_true', help='Save occupancy/poses')
    ap.add_argument('--plan', action='store_true', help='Run A* on the occupancy grid for a random goal')
    ap.add_argument('--engine', choices=['auto','2d','pybullet'], default='auto', help='Simulation engine')
    ap.add_argument('--rays', action='store_true', help='Draw LiDAR rays in PyBullet GUI')
    args = ap.parse_args()

    DT = 0.1
    angles = np.linspace(-math.pi/2, math.pi/2, 121)

    # Select engine
    engine = args.engine
    if engine == 'auto':
        engine = 'pybullet' if HAVE_PYBULLET else '2d'
    if engine == 'pybullet' and not HAVE_PYBULLET:
        print('[!] PyBullet not available — falling back to 2D engine.')
        engine = '2d'

    if engine == 'pybullet':
        world = _World3D(gui=True, seed=0)
        world.create_robot()
        max_range = world.max_range
    else:
        world = SimpleWorld2D(seed=0, size=6.0, max_range=5.0)
        max_range = world.max_range

    robot = DiffDriveRobot(x=-2.0, y=-2.0, theta=0.0)
    occ = OccupancyGrid(size_m=8.0, resolution=0.05)
    pf = ParticleFilter(occ, n_particles=400, init_pose=robot.pose(), trans_noise=0.03, rot_noise=0.02)

    logger = RunLogger(root='runs') if (args.record or args.save) else None

    plt.ion()
    fig, ax = plt.subplots()
    im = ax.imshow(occ.prob(), origin='lower', cmap='gray', vmin=0.0, vmax=1.0)
    scat_parts = ax.scatter([], [], s=2, c='red')
    scat_robot = ax.scatter([], [], marker='x', c='blue')
    path_plot = None
    title = f'Tiny-SLAM Turtle ({engine.upper()}) : Occupancy + Particles (+ A*)'
    ax.set_title(title)
    fig.tight_layout()

    t = 0.0; last_vis = 0.0; step = 0
    goal = np.array([2.0, 2.0])
    planned_cells = []

    while t < args.seconds:
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
        z = np.clip(z + np.random.normal(0, 0.01, size=z.shape), 0.0, max_range)

        # Update PyBullet robot pose and optional beams
        if engine == 'pybullet':
            world.set_robot_pose(*robot.pose())
            if args.rays:
                world.show_rays(robot.pose(), angles, z)

        occ.update_from_scan(robot.pose(), z, angles, max_range)

        pf.predict(v=v, omega=omega, dt=DT)
        pf.update(z_obs=z, angles=angles, max_range=max_range, sigma=0.08)
        pf.resample()
        est = pf.estimate()

        if args.plan and step % 10 == 0:
            prob = occ.prob()
            occ_bin = (prob > 0.65).astype(np.uint8)
            rx, ry = occ.world_to_map(x, y)
            gy, gx = choose_random_free(occ_bin)
            path_cells = astar(occ_bin, (ry, rx), (gy, gx), allow_diag=True)
            planned_cells = path_cells or []

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
            if args.plan:
                if path_plot is not None: path_plot.remove()
                if planned_cells:
                    ys, xs = zip(*planned_cells)
                    path_plot = ax.plot(xs, ys, linewidth=2)[0]
                else:
                    path_plot = None
            plt.pause(0.001); last_vis = t

            if logger:
                if args.save:
                    logger.save_occ(prob, step)
                    logger.save_pose(robot.pose(), est, step)
                if args.record:
                    logger.save_frame(fig, step)
            step += 1

        t += DT

    if logger and args.record:
        out = logger.try_make_video(fps=10)
        if out: print(f'Wrote video: {out}')
        else: print('Saved frames; install imageio-ffmpeg to auto-write MP4.')

    print('Done. Close the plot to exit.')
    plt.ioff(); plt.show()
    world.disconnect()

if __name__ == '__main__':
    main()
