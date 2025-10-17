# Lidar-Lite SLAM: Real-Time Mapping, Localization, and Path Planning

Lidar-Lite SLAM is a compact robotics simulation that demonstrates how a mobile robot can sense, map, and navigate its surroundings; using nothing more than a simulated LiDAR and probabilistic algorithms.
The project shows the three key parts of autonomy: mapping, localization, and path planning, all written from scratch in Python and visualized in real time.

What This Project Does?

The goal is to replicate the behavior of an autonomous ground robot exploring an unknown space, similar to a TurtleBot or warehouse robot.
The robot doesn’t know the environment in advance. It starts with a blank map and gradually builds an understanding of the world around it using LiDAR.

Here’s what happens:

- The robot drives around the environment using simple differential drive motion.
- A LiDAR scanner mounted on the robot continuously sends out range beams in a semicircular pattern.
- Each LiDAR beam measures how far it travels before hitting an obstacle.
- These readings are converted into an occupancy grid map, where:
- White regions represent walls or obstacles,
- Black regions represent free space, and
- Gray regions represent unknown areas that haven’t been scanned yet.
- At the same time, a particle filter estimates the robot’s position and orientation by maintaining hundreds of hypotheses (particles) that evolve based on motion and sensor noise.
- As the robot learns the map, an A* path planner computes routes between its current location and random goals within the known area.
- Over time, the map becomes more detailed, and the robot localizes itself with increasing accuracy, just like a real SLAM system.
