# Autonomous Drone

Python implementation of a quadrotor autonomy pipeline for simulation. The
repository combines 3D occupancy-map generation, graph-based path planning,
trajectory generation, SE(3) control, and visual-inertial odometry components
used to navigate a drone through obstacle-filled environments.

The code is organized around the main subsystems of an autonomous flight stack:

- `occupancy_map.py`: builds a voxelized configuration-space map from a
  simulated world and obstacle geometry.
- `graph_search.py`: runs 3D graph search over the occupancy grid to find a
  collision-free path between start and goal positions.
- `world_traj.py`: converts the planned path into waypoint and minimum-jerk
  trajectory segments for execution.
- `se3_control.py`: implements an SE(3)-style controller that maps desired flat
  outputs and current state into thrust, moment, and motor speed commands.
- `vio.py`: contains nominal-state propagation, covariance prediction, and
  measurement update steps for visual-inertial odometry.
- `sandbox.py`: local experimentation entry point for running and testing the
  modules together.

This project is structured for simulation-based development and coursework in
robotics, controls, and autonomous aerial navigation.
