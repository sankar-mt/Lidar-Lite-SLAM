# Tiny-SLAM Turtle (Python)

A from-scratch, laptop-friendly robotics project demonstrating **mapping**, **localization**, and **perception** in simulation — no ROS required.

## What it does
- Simulates a simple differential-drive robot in **PyBullet**
- Builds an **occupancy-grid map** from a LiDAR-like ray sensor
- Estimates robot pose using **Monte Carlo Localization (MCL)**
- (Optional) Uses **OpenCV** perception hooks (ArUco/feature) to correct drift
- Visualizes map + particles live in **Matplotlib**

> This project is intentionally original and **not derived from any coursework**. All code here is standalone and built for learning + portfolio use.

## Quick Start
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python run.py
```
