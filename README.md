# Autonomous VIO based Quadrotor

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![GitHub Issues](https://img.shields.io/github/issues/ayushgoel24/Autonomous-VIO-based-Quadrotor.svg)](https://github.com/ayushgoel24/Autonomous-VIO-based-Quadrotor/issues)
[![Contributions welcome](https://img.shields.io/badge/Contributions-welcome-orange.svg)](https://github.com/ayushgoel24/Autonomous-VIO-based-Quadrotor)

This repository contains code for implementing a nonlinear geometric controller and generating an obstacle-free path for a quadrotor. The project utilizes the A* algorithm for path planning and the minimum-snap trajectory generator for smooth and dynamically feasible trajectory generation.

## Overview

The goal of this project is to develop a robust control system for a quadrotor, enabling it to navigate from a given start position to a goal position while avoiding obstacles. The implementation consists of the following key components:

1. **Geometric Controller:** A nonlinear geometric controller is implemented to determine the control commands, specifically the thrust and moment, required to track the desired trajectory. A PD controller is employed to calculate the accelerations and moments necessary for reaching a specific position and orientation.

2. **A* Path Planning:** The A* algorithm is utilized to generate an obstacle-free dense set of points from the start position to the goal position. The L2 norm from each node to the goal node serves as the heuristic for the A* algorithm, ensuring efficient path selection.

3. **Ramer-Douglas-Peucker Downsampling:** Once the obstacle-free dense set of points is obtained, the Ramer-Douglas-Peucker downsampling algorithm is applied to identify the optimal number of points required for the path. This process helps reduce computational complexity while maintaining the essential waypoints.

4. **Minimum-Snap Trajectory Generation:** The downsampled set of points is then passed to the minimum-snap trajectory generator. This generator is responsible for creating a smooth and dynamically feasible trajectory for the quadrotor, minimizing jerk (rate of change of acceleration) along the path.

5. **Visual Inertial Odometry:** The code also includes the fusion of IMU measurements and stereo pair camera images to estimate the pose of the quadrotor. The Error State Kalman Filter (ESKF) is employed for visual inertial odometry, providing accurate pose estimation during flight.

## Getting Started

To get started with the code, please follow these instructions:

1. Clone the repository to your local machine.
2. Install the necessary dependencies specified in the requirements file.
3. Run the main script, `main.py`, to execute the quadrotor control system and path planning algorithms.
4. Adjust the desired start and goal positions, as well as any other relevant parameters, in the configuration file, `config.yaml`.
5. Observe the quadrotor's trajectory and performance as it navigates from the start position to the goal position, avoiding obstacles along the way.

## Results

<table>
    <tr>
        <td align = "center"> <img src="./results/path_1.gif"></td>
        <td align = "center"> <img src="./results/path_2.gif"></td>
        <td align = "center"> <img src="./results/path_3.gif"></td>
    </tr>
    <tr>
        <td align = "center">Trajectory 1</td>
        <td align = "center">Trajectory 2</td>
        <td align = "center">Trajectory 3</td>
    </tr>
</table>

## Acknowledgments

We would like to acknowledge the following resources and references that contributed to the development of this project:

- [A* Pathfinding Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Ramer-Douglas-Peucker Algorithm](https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm)
- [Minimum-Snap Trajectory Generation for Quadrotors](https://ieeexplore.ieee.org/abstract/document/5980409)
- [Error State Kalman Filter (ESKF)](https://ieeexplore.ieee.org/abstract/document/5980409)