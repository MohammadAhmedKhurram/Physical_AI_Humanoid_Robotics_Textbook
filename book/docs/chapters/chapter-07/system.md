System

A practical SLAM system for humanoids is layered: sensor fusion at the lowest layer (cam+IMU+contact), a visual odometry front‑end, a back‑end for optimization and loop closure, and a map server that publishes both sparse graphs and dense occupancy maps. Middleware (ROS 2) facilitates data flow: camera images, IMU, joint states, and foot force readings are timestamped and synchronized (use ROS 2 simulated/robot clock). The system must support high‑rate IMU updates (≥200 Hz) while keeping visual processing in parallel pipelines.

Critical components:
- Sensor synchronization and calibration (Kalibr)
- VIO / Visual odometry front‑end (ORB‑SLAM3, VINS‑Mono)
- Back‑end optimization (g2o, Ceres, GTSAM)
- Map server (sparse pose graph + dense OctoMap/TSDF)
- Footstep planner interface and safety supervisor

Physically grounded example: implement an RTAB‑Map pipeline running on a humanoid where stereo images are rectified and passed to ORB feature extractors; IMU preintegration provides priors to a sliding window estimator, and foot contact triggers zero‑velocity updates. The back‑end runs pose graph optimization (g2o/Ceres) and the map server produces both a sparse pose graph for loop closures and a dense OctoMap for footstep planning. Time‑synchronization, calibration, and failure recovery policies are part of the system requirements.

Operational considerations and engineering design patterns:
- Use message filters and hardware timestamps where possible to reduce synchronization errors. If hardware timestamps are unavailable, instrument a synchronization node that estimates timestamp offsets and publishes corrected headers.
- Maintain a manifest for calibration artifacts (camera intrinsics, camera‑IMU extrinsics from Kalibr) and include versioned JSON pointers in the robot repository.
- Adopt resource budgeting: run VIO and feature extraction on a dedicated real‑time core or GPU, and offload pose graph optimization to a non‑real‑time worker to avoid control path jitter.

Trade‑offs and invariants: dense volumetric maps are useful for footstep planning but expensive in memory; sparse graphs are compact and support long‑term loop closures. Ensure a clear contract between the SLAM map representation and the footstep planner to avoid costly conversions at runtime. Always provide fallbacks when visual tracking fails (e.g., odometry + kinematics fallback).