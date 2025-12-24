---
title: Chapter 05 — Digital Twins with Gazebo & Unity
---

## Concept

A digital twin is a computationally faithful representation of a physical robotic system and its operating environment: geometry and inertial properties, actuator and sensor models, on-board software, and state history. In robotics, a digital twin enables physics‑aware testing, perceptual pipeline development, data augmentation for learning, and operator training without exposing hardware to risk. The twin must capture three tightly coupled facets: (1) kinematics and dynamics (mass, inertia, joint limits, actuator dynamics), (2) perceptual fidelity (sensor geometry, optics, noise, latency), and (3) software parity (control loops, middleware topics, parameter sets). Together they permit experiments that are reproducible, auditable, and instrumented with ground truth that is rarely available on the physical platform.

Physically grounded example: simulate a TurtleBot‑style mobile base in Gazebo with a simulated 2D lidar (Hokuyo model), IMU, and wheel encoders while mirroring the same robot pose and camera intrinsics into Unity for photorealistic RGB capture. Use the Gazebo physics engine for contact, friction, and dynamics; use Unity for high‑quality imagery used to train a perception network. This split leverages each tool’s strengths while the system-level twin preserves state consistency through ROS 2 messaging.

Key conceptual constraints: a twin is not merely visualization — it is an executable specification of system behavior that must be validated against the real system continuously. Sources of mismatch (model error, unmodelled friction, sensor calibration drift, network latency) must be explicitly tracked and quantified. Chapter acceptance focuses on how to create twins that are useful: accurate enough to expose integration faults and fast enough for iterative development and dataset generation.


## System

A practical digital‑twin system for robotics is layered and service oriented. At the lowest layer sits a fidelity‑tunable physics engine (Gazebo or NVIDIA Isaac) that executes rigid‑body dynamics, contact, and low‑level actuator models. A sensor layer implements geometric and stochastic sensor models (lidar raycasts, pinhole or path‑traced camera models, IMU noise models). A middleware layer — typically ROS 2 using DDS — provides the communication substrate for state synchronization, command streams, and telemetry. A rendering layer (Unity or Isaac’s RTX‑accelerated renderer) produces photorealistic frames when visual realism is required for perception training or human‑in‑the‑loop simulation. Service components provide bridging (e.g., ros_gz_bridge or ROS↔Unity connectors), state stores (time‑series databases or rosbag archives), and orchestration (containerized nodes, launch descriptions).

Physically grounded example: architect a pipeline where Gazebo runs the dynamic simulation and publishes joint states and TF over ROS 2; a ros_gz_bridge node translates Gazebo topics into ROS 2 topics. A separate Unity process, connected via the Unity Robotics Hub (ROS‑TCP‑Connector), subscribes to the camera TF and publishes RGB frames for model training. Time synchronization is enforced using simulated ROS 2 clocks and host NTP/Chrony for hardware timestamps to bound drift.

Architectural trade‑offs revolve around fidelity versus throughput: high‑fidelity path tracing in Unity or Isaac Sim yields excellent images for perception but demands GPU resources and slows iteration; simpler pinhole camera models in Gazebo scale easily but lack photorealism. Design the twin as a composition of targeted fidelity domains rather than a monolithic high‑fidelity stack.


## Simulation

Simulation is an experiment design activity: choose what to simulate, how deterministic the run must be, and how to represent uncertainty. Important knobs include physics timestep, contact solver parameters, friction and restitution coefficients, actuator bandwidth, sensor sampling rates, and noise/latency models. Reproducible experiments require deterministic seeds, containerized environments, and recorded configuration manifests (URDF/SDF, plugin parameters, ros2 launch files). For perception, couple a physics simulator (Gazebo or Isaac) to a rendering engine (Unity or Isaac RTX) to produce ground‑truth labels: object poses, segmentation masks, depth, and per‑pixel semantic annotations.

Physically grounded example: configure a Gazebo scenario emulating a warehouse aisle where a mobile robot navigates between shelves. Use an SDF world with static obstacles and an IMU + lidar sensor set on the robot model. Run Monte Carlo trials with domain randomization: vary friction, payload mass, lidar reflectivity, and lighting conditions in Unity for RGB renders. Record rosbag2 archives of each run (joint states, odometry, lidar point clouds, camera frames) to build training datasets and to quantify navigation robustness. Use deterministic seeds for the physics engine to enable statistical comparison between experiments.

Validation experiments should compare simulation outputs with a short real‑world run: compare odometry drift, lidar point patterns, and latency to tune noise models. When mismatch exceeds thresholds, flag the twin model for recalibration.


## Implementation

Implementing a digital twin is an engineering pipeline that moves from specification to executable model. Steps: (1) formalize robot geometry and joint limits in URDF/Xacro and export an SDF for Gazebo if required; (2) select sensor plugins (lidar rays, camera sensors, IMU) and parameterize noise and latency; (3) integrate control abstraction via ros2_control so the same controllers can bind to hardware or simulation controllers; (4) provision bridges between simulators and other tooling (ros_gz_bridge for Gazebo, ROS‑TCP‑Connector for Unity, or the native Isaac Sim ROS bridge); (5) create validation harnesses that replay rosbag2 data to assert parity metrics (pose RMSE, perception precision/recall, closed‑loop stability).

Physically grounded example: to twin a 6‑DOF manipulator, author a URDF with collision and inertial parameters, enable joint effort/position controllers using ros2_control, attach a simulated Intel RealSense camera in Gazebo for depth and RGB, and mirror frames into Unity for photorealistic image capture. Implement a node that computes per‑step pose error between the real manipulator and the simulated state; use these metrics to update actuator friction and gear ratio parameters iteratively.

Implementation must avoid ad‑hoc sync: centralize time with ROS 2 simulated clock when possible, and adopt well‑documented bridge contracts so datasets and pipelines remain reproducible. Automate instantiation with launch descriptions and container images to reduce configuration drift.


## Integration

Integrating digital twins into engineering workflows makes them operational: continuous dataset generation, CI‑driven regression tests, hardware‑in‑the‑loop (HIL) stages, and verification loops for parameter calibration. Typical integration points include: (a) a CI job that runs headless Gazebo scenarios and verifies navigation metrics; (b) a data pipeline that ingests rosbag2 artifacts, extracts training examples in Unity/Isaac, and publishes datasets to a model training registry; (c) an HIL bench where commands from a simulated controller exercise real actuators through ros2_control hardware interfaces under constrained conditions.

Physically grounded example: configure a GitHub Actions workflow that brings up a container with headless Gazebo and a minimal ROS 2 stack, executes 100 randomized navigation trials, stores rosbag2 outputs as artifacts, and runs a validation job that asserts odometry RMSE < X meters and collision count = 0 for the baseline policy. For vision models, use Unity or Isaac Sim to produce photorealistic imagery in a GPU‑enabled runner and push resulting datasets to a training pipeline. HIL testing can be performed by replaying rosbag2 sensor streams into the real robot while gating actuator commands behind safety monitors.

Integration success criteria emphasize reproducibility, measurable parity metrics between sim and real, and automated gating so that model or controller changes cannot degrade safety or performance unnoticed.
