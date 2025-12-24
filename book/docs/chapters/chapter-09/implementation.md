---
title: "Chapter 9 — Manipulation & Grasping: Implementation"
tab: Implementation
word_count: 393
---

Implementation focuses on concrete software and hardware practices that realize manipulation pipelines. Use ROS 2 as middleware, MoveIt 2 for planning and grasp execution, and ros2_control for hardware abstraction and controller deployment. Structure packages clearly: perception (segmentation, pose estimation), grasp (candidate generation, ranking), planning (MoveIt configuration and motion planners), control (controller configs and plugins), and experiments (scripts and datasets).

Perception implementation: use PCL and Open3D for point-cloud processing, including filtering, voxel downsampling, plane segmentation (RANSAC), and clustering. For pose estimation of known objects, integrate iterative closest point (ICP) or use learning-based pose estimation networks (DenseFusion, PVNet). Publish results on /detected_objects with pose and bounding information.

Grasp synthesis: analytic approaches compute antipodal contacts and force-closure measures, while data-driven models (Dex-Net, GPD, or custom CNNs/PointNet-based models) provide robust sampling in cluttered scenes. Use a grasp candidate message type that includes pose, approach vector, grasp width, and a confidence metric. Implement candidate ranking using expected wrench-space margin or learned value functions.

Motion planning: configure MoveIt 2 with the robot's URDF and SRDF, set up planning groups for arms and grippers, and tune planners (OMPL) for constrained approach motions. Implement pre-grasp offsets and approach/retract stages; set collision checking tolerances and allowed collisions for objects the gripper encloses.

Controllers: implement impedance/force controllers in ros2_control or extend existing controller interfaces for hybrid force/position control. Employ low-level safety monitors that subscribe to joint/FT topics to abort or switch to safe modes on threshold violations. For grippers, implement state machines for open/close with position and force goals.

Example — Implementation on a Franka robot: Create ROS 2 packages: franka_description (URDF), franka_moveit_config, perception_pipeline, grasp_server, and franka_controllers. Configure ros2_control for joint trajectory and impedance controllers, and write an action server /execute_grasp that sequences perception, grasp selection, MoveIt planning, and controller transitions. Use continuous integration to run static checks and linter rules, and unit tests for message contracts and small processing functions.

Acceptance checks:
- [ ] Each tab file exists at /book/docs/chapters/chapter-09/ and contains metadata
- [ ] Each tab has 350–450 words
- [ ] Each tab includes a labeled Example section

Follow-ups and risks:
- Data drift between sim and reality may require dataset augmentation
- Safety during hardware tests requires stringent interlocks and test harnesses
- Complex, deformable objects remain an open challenge for analytic grasps
