---
title: "Chapter 9 — Manipulation & Grasping: System"
tab: System
word_count: 391
---

An engineering system for manipulation comprises hardware, perception, planning, and control modules organized by clear interfaces and data flows. Hardware includes the robot arm (6–7 DOF serial manipulators such as Franka Emika Panda or Kinova Gen3), end-effector (parallel-plate or multi-finger hands such as Robotiq 2F/Hand-E), and sensing suite (wrist F/T sensor, fingertip force sensors, RGB-D camera like Intel RealSense, and optionally tactile arrays). A modular software stack uses ROS 2 for middleware, MoveIt 2 for motion planning, and real-time controllers (e.g., ros2_control with ros2_controllers) for low-level actuation and impedance control.

The system architecture separates concerns across layers: perception (point cloud acquisition, segmentation, pose estimation), grasp synthesis (analytic wrench-space or data-driven sampling), motion planning (collision-aware whole-body planning), and control (hybrid force/position and impedance controllers). Communicate across ROS 2 topics and services: /camera/points (sensor_msgs/PointCloud2), /grasp_candidates (custom message with pose and metric), /execute_grasp (action server for coordinated approach-grasp-lift), and /gripper/command (controller interface) for actuation.

State machines or behavior trees orchestrate phases: perceive → plan → approach → contact → lift → verify → place. Use BehaviorTree.CPP or SMACC in ROS 2 to coordinate retries and fallbacks (e.g., reattempt with alternate grasp or perform handoff to another manipulator). Safety interlocks monitor torque and F/T readings, halting motion on anomalous thresholds.

Calibration and frame transforms are critical: maintain TF tree accuracy between camera, wrist, and end-effector frames. Use ROS 2 calibration tools (e.g., industrial_calibration) to compute extrinsics. For hardware, implement compliance using torque control or series-elastic actuators to reduce impact forces during contact. In simulation, replicate compliance via Gazebo plugins or ros2_control SimHw interfaces to validate controller behavior before hardware trials.

Example — System integration on a real robot: For a pick-and-place pipeline on a Franka Emika Panda with Robotiq gripper, mount an Intel RealSense D435 on the wrist. Perception node segments objects and computes grasp candidates; MoveIt 2 plans approach trajectories while ros2_control runs an impedance controller to softly make contact. A wrist F/T sensor detects contact and triggers a transition to force control during insertion tasks. Logging and playback systems capture sensor streams (rosbag2) to iterate grasp policies offline.

Design the system for observability and iterative improvement: record metrics (grasp success rate, slip events, force overshoot), visualize candidate distributions in RViz, and maintain experiment datasets for retraining data-driven grasp models.