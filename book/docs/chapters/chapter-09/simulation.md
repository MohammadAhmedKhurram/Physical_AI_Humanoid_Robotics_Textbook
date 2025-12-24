---
title: "Chapter 9 — Manipulation & Grasping: Simulation"
tab: Simulation
word_count: 405
---

Simulation is central to developing and validating manipulation strategies before hardware trials. High-fidelity simulation captures contact physics, sensor noise, and controller timing to enable rapid iteration. Use Gazebo (with ODE or Bullet), PyBullet, or NVIDIA Isaac Sim depending on fidelity needs: Isaac Sim provides advanced contact models and GPU-accelerated physics, while Gazebo integrates tightly with ROS 2 and MoveIt 2 for end-to-end pipelines.

Key simulation elements: accurate URDF/SDF models with collision geometry and inertial properties, joint and actuator limits, gripper models with contact geometries, and tactile sensor plugins (e.g., Gazebo tactile plugins or Isaac Sim tactile sensors). Model friction properties and contact compliance explicitly; real-world grasps often succeed due to compliance that is absent in rigid simulations. Add sensor noise models for depth cameras and tactile sensors, and calibrate the noise parameters by comparing logs from hardware to simulation.

Simulation workflows should mirror the real system: run perception stacks that consume simulated point clouds (from depth camera plugins), feed grasp proposal networks or analytic grasp planners, and execute planned trajectories via simulated ros2_control controllers. Validate controller transitions (free-space → contact) in simulation using a wrist F/T sensor plugin and closed-loop impedance controllers. For data-driven methods, gather large-scale synthetic datasets in simulation (annotated grasps, contact patches, and failure cases) to train and validate grasp predictors like Dex-Net or GPD.

Hardware-in-the-loop (HIL) and sim-to-real pipelines reduce the domain gap: perform HIL by running real controllers or force/torque sensors against simulated environments, or collect paired datasets where the same pick is executed in sim and on hardware to compute correction models. Domain randomization — varying friction, mass, sensors, and lighting — improves robustness of learned policies by exposing models to a wider observation distribution.

Example — Gazebo + ROS 2 pick pipeline: Create an SDF world with table and objects, spawn a URDF model of a manipulator with a Robotiq gripper, attach an Intel RealSense depth camera plugin to the robot wrist, and run the perception nodes to publish PointCloud2 to /camera/points. Run a grasp-sampling node to propose grasps, execute with MoveIt 2 through ros2_control simulated controllers, and log outcomes in rosbag2 for dataset curation. Use parameter sweeps over friction coefficients to stress-test grasp strategies.

Simulation is not a perfect substitute for hardware, but with careful modeling, noise injection, and HIL practices it becomes an indispensable tool for safe, repeatable development of manipulation capabilities.