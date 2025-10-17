# Tiny-SLAM Turtle (Python)

A from-scratch, laptop-friendly robotics project demonstrating **mapping**, **localization**, and **perception** in simulation ; no ROS required.

## What it does
- Simulates a simple differential-drive robot in **PyBullet**
- Builds an **occupancy-grid map** from a LiDAR-like ray sensor
- Estimates robot pose using **Monte Carlo Localization (MCL)**
- Uses **OpenCV** perception hooks (ArUco/feature) to correct drift
- **A*** path planning over the occupancy grid
- **recording** of frames and auto MP4 if `imageio-ffmpeg` is available
- Visualizes map + particles live in **Matplotlib**

> This project is intentionally original and **not derived from any coursework**.

## Quick Start
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python run.py --seconds 45 --record --save --plan
```

### Useful Flags
- `--seconds N` : run duration (default 60s)
- `--record`    : save frames each tick and try to write `runs/<ts>/demo.mp4`
- `--save`      : save occupancy maps and poses under `runs/<ts>/`
- `--plan`      : run A* from current robot cell to a random free goal cell and render path

## Dependencies
```
numpy
matplotlib
pybullet
opencv-python
opencv-contrib-python  # only if you enable ArUco
imageio                # for saving frames/videos
imageio-ffmpeg         # (optional) write MP4 automatically
```


## Project Name
**Tiny‑SLAM Turtle** — a compact end-to-end SLAM demo (mapping + MCL + A*).

## See the robot move in 3D (PyBullet)
```bash
python run.py --engine pybullet --plan --rays
# Matplotlib: map + particles; PyBullet: blue cylinder robot moving with optional LiDAR rays
```
If PyBullet isn’t available, use the 2D engine:
```bash
python run.py --engine 2d --plan
```
