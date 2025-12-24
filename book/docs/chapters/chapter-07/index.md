---
title: Chapter 07 — Visual SLAM & Navigation for Humanoids
---

## Concept

Visual simultaneous localization and mapping (Visual SLAM) for humanoid platforms is the study of producing a geometrically and temporally consistent spatial model together with a tightly‑coupled, drift‑bounded pose estimate during highly non‑stationary, gait‑driven motion. Unlike wheeled robots, humanoids present articulated sensor rigs (head, torso, limb‑mounted cameras), intermittent and asymmetric contact sequences (single‑support, double‑support, impact), and large‑amplitude viewpoint changes from head movements and manipulation. These factors make observability, measurement modelling, and map representation central design decisions.

A humanoid SLAM architecture must reconcile two overlapping objectives: (1) low‑latency short‑term state estimation to support balance and reactive footstep control, and (2) medium‑ to long‑term mapping and loop closure for reliable navigation and mission planning. Visual‑inertial odometry (VIO) or sliding‑window estimators provide the high‑rate pose priors required for balance controllers, while a pose‑graph or submap‑based back‑end enforces global consistency and supports relocalization. Contact sensing (foot force/torque, joint encoders) offers additional kinematic constraints that can reduce drift during contact phases; zero‑velocity or kinematic priors derived from contacts are valuable when visual observability degrades.

Map representation is a crucial choice: sparse feature graphs (keyframes + landmarks) are memory efficient and support robust loop closure, whereas denser surfel/TSDF/OctoMap representations are necessary for granularity in footstep planning, terrain assessment, and manipulation. Sensor choice (monocular, stereo, RGB‑D, event cameras) determines scale observability and robustness to motion blur; for humanoids a tightly‑coupled visual‑inertial approach with depth augmentation is typically most resilient.

Acceptance checks (Concept):
- [ ] Describe VIO vs global mapping tradeoffs for humanoids
- [ ] List concrete tools and sensors (e.g., ROS 2, ORB‑SLAM3, RTAB‑Map, Intel RealSense, VectorNav)
- [ ] Provide at least one grounded, physically realistic example with expected outputs

### Example

A head‑mounted Intel RealSense D435 coupled with a VectorNav VN‑100 IMU running a tightly‑coupled ORB‑SLAM3 VIO frontend on ROS 2 yields a robust short‑term pose for balance while a background pose‑graph (g2o/Ceres) performs loop closure. The system exports keyframes, a pose graph, and a fused OctoMap for footstep planning used by a planner.

## System

A practical humanoid Visual SLAM system is organized into modular layers: sensors and timing, preprocessing, front‑end state estimation, back‑end optimization and mapping, and an integration API for planners and controllers. At the lowest level, accurate time synchronization and transform consistency (tf2) are mandatory: the camera frame must be registered to base_link through URDF and a joint_state_publisher so that head motions are accounted for in per‑frame transforms. Prefer hardware timestamps for cameras and IMUs; when unavailable, a synchronization node that estimates offsets and corrects headers reduces systematic timing error.

Architectural invariants include deterministic latency for the localization path (often ~100 ms), a clear contract between sparse and dense map representations, and explicit resource budgets per computation domain (real‑time VIO vs non‑real‑time global optimization). Multi‑rate fusion is implemented with IMU preintegration algorithms (GTSAM preintegrated factors) or MSCKF/sliding‑window estimators; foot contact provides zero‑velocity priors or kinematic constraints during single‑support phases and should be gated to avoid false constraints during slipping.

Operational patterns emphasize compute partitioning (dedicated real‑time core or GPU for VIO, worker node for pose‑graph), DDS QoS tuning for ROS 2 topics under constrained networks, and hierarchical map management (local submaps rolled up into a global pose graph) to bound memory and compute. Monitoring and metrics are essential: track estimator covariance growth, loop closure proposal and verification rates, CPU/GPU utilization, and per‑topic latencies. Map versioning and rollbacks are operational safeguards when planner behavior depends on evolving map content.

Engineering controls for fielded humanoids include: (a) deterministic thread affinity and CPU isolation for the localization pipeline; (b) hardware triggers or PTP clocking when available to remove software timestamp jitter; (c) a message_filter and exact_time policy for aligning image, IMU and joint messages when hardware timestamps are absent. These measures reduce timing jitter that otherwise amplifies during impact events such as foot strikes.

System design should also define clear failure modes: when visual tracking becomes unreliable due to low texture or severe motion blur, the stack must gracefully degrade to contact‑augmented kinematic odometry with conservative covariance inflation, and notify the mission planner to reduce gait aggressiveness.

### Example

Deploy a split compute stack where a Jetson AGX Xavier performs GPU‑accelerated feature extraction and VIO on ROS 2, and an Intel NUC runs pose‑graph optimization and an OctoMap server. Use tf2 to publish transforms, record calibration artifacts (camera_info topics and Kalibr outputs) in a versioned manifest, and tune DDS QoS to prefer timely delivery of image and IMU topics for the localization pipeline.

## Simulation

Simulation is a central engineering tool for iterating humanoid Visual SLAM before hardware experiments. Two complementary simulator classes should be used in tandem: physics‑accurate engines (Gazebo / Ignition) to reproduce kinematics, contact forces, and controller interactions; and photorealistic rendering platforms (Unity, NVIDIA Isaac Sim) to exercise visual nuisances such as lighting change, motion blur, rolling shutter, and reflective surfaces. A combined workflow uses Gazebo for controller/HIL fidelity and Isaac Sim or Unity for perceptual realism and domain randomization.

A robust simulation pipeline includes realistic sensor plugins (camera and IMU models with configurable noise densities, rolling vs global shutter, and motion blur), accurate contact models (friction, compliance, foot shape), and scene libraries that reflect deployment geometries. Use ros_gz or native ROS 2 bridges to expose sensor topics (sensor_msgs/Image, sensor_msgs/Imu, joint_states, and custom foot_contact messages) and record deterministic rosbag2 datasets with run manifests and seeds to reproduce trials.

For perception stress tests, use Isaac Sim or Unity to render photoreal frames with domain randomization applied to textures, illumination, and dynamic actors. Export synchronized RGB, depth, and normal maps for training or validating learned components such as dynamic object segmentation masks and depth refinement networks. For controller and timing validation, rely on Gazebo HIL benches where URDF controllers and ros2_control plugins run to expose true‑to‑hardware timing and feedback.

Automation and validation practices include Monte Carlo sweeps across gait parameters (step length, cadence, payload) and environment variables (lighting, texture repetitiveness). Containerized batch runs generate hundreds of short trials; aggregate metrics (ATE, RPE, loop closure recall, false positive rate) to evaluate robustness. Always validate simulator parameterization against short real‑world sequences (motion capture / Vicon) and adjust IMU/contact noise models until parity metrics are within acceptable bounds.

To support reproducibility and continuous validation, record and publish per‑trial manifests that include the simulator version (Gazebo/Isaac), URDF/USD asset checksums, Kalibr artifacts, and random seeds. Define pass/fail criteria for automated trials (for example: ATE < 0.5 m on indoor stair sequences, loop closure recall > 90% on repeated corridors, bounded estimator covariance growth) and fail experiments that exceed thresholds. These artifacts enable reproducible regression tests in CI and simplify parameter tuning when sim‑to‑real gaps are discovered.

### Example

Create a Gazebo scenario with a humanoid URDF and attach a simulated Realsense RGB‑D camera and IMU. Script a stair ascent with randomized lighting and friction perturbations, record rosbag2, and replay through ORB‑SLAM3 and RTAB‑Map on ROS 2 to measure trajectory RMSE and loop closure recall. Export photoreal frames from Isaac Sim for perceptual training where needed.

## Implementation

A staged implementation strategy reduces risk and improves repeatability: (1) collect calibration artifacts and verify their application, (2) ensure drivers and synchronization are robust, (3) implement and optimize a front‑end VIO tuned for low latency, (4) deploy a back‑end pose‑graph and map server, (5) integrate contact priors and perform HIL validation. Calibration is foundational — record Kalibr sequences, compute camera intrinsics and camera‑to‑IMU extrinsics, and commit artifacts with version metadata and checksums so deployments can reproduce exact parameters.

Driver implementations must prefer hardware timestamps and expose camera_info topics compatible with camera_info_manager patterns on ROS 2. If hardware timestamps are unavailable, implement a synchronization node that estimates per‑sensor offsets via correlation methods and corrects headers before the estimator consumes messages. For front‑end performance, profile feature detectors and descriptor extraction; offload descriptor computation to CUDA/TensorRT on Jetson or use OpenCV multithreading on NUC‑class hardware to meet real‑time budgets.

Back‑end design should focus on robust loop closure verification: combine NetVLAD/DBoW2 place proposals with geometric verification to avoid false positives, and adopt hierarchical optimization where local submaps are optimized frequently and aggregated into a global pose graph at lower frequency. Contact integration should provide gated zero‑velocity priors or kinematic constraints derived from foot force sensors and joint encoders; gate constraints when contact quality metrics fall below thresholds to prevent corrupting the estimator during slips.

Testing and tooling: automate rosbag2 replay tests with ground truth comparisons (ATE/RPE), create parameter sweep harnesses for key thresholds (feature matching ratio, keyframe insertion interval), and surface performance metrics (latency, CPU/GPU, estimator covariance) in dashboards. Short HIL acceptance tests on a bench should assert estimator latency and loop closure reliability before extended deployments.

### Example

Run Kalibr sequences with head pan/tilt and base motions to compute extrinsics; apply results to the URDF and SLAM configuration, then replay a rosbag2 on ROS 2 while measuring ATE against Vicon ground truth. Deploy VIO on a dedicated real‑time core (Jetson) and the pose‑graph on a worker node, and execute HIL acceptance walks to confirm latency and loop closure performance.

## Integration

Integration operationalizes SLAM outputs for footstep planning, whole‑body control, and mission autonomy. Define precise APIs and data contracts: a localization topic publishing pose + covariance, map export endpoints for keyframes and dense layers, and incremental map snapshot semantics consumable by footstep planners. Planners must incorporate estimator uncertainty via covariance gating, and supervisor nodes must enforce reachability and dynamic stability checks before admitting execution commands to ros2_control controllers.

Choose an integration pattern that matches operational risk: loose coupling (SLAM publishes maps and pose; planner queries snapshots) simplifies failure isolation and testing, whereas tight coupling (planner constraints embedded within SLAM optimization) can reduce conservatism for complex maneuvers but increases real‑time coupling and testing complexity. Regardless of pattern, design clear failure handlers: fall back to contact‑augmented kinematic odometry, reduce gait aggressiveness, and notify operators through telemetry channels when localization confidence falls below thresholds.

Operationalizing maps requires robust versioning and rollback semantics. Exported maps and snapshots must include creation metadata, algorithm/version identifiers, URDF and calibration checksums, and provenance for the simulation or hardware run used to generate them. Implement atomic map update semantics so planners never observe partially‑written artifacts; provide a map registry that supports rollbacks to a known‑good map and tracks performance deltas between versions. For safety, require verified sanity checks (map coverage, heightfield consistency) before a new map becomes the production snapshot consumed by planners.

CI and HIL harnesses should replay representative rosbag2 streams and assert localization drift, step success rates, and safety trip incidence to gate merges. Instrument dashboards that surface localization error statistics, step success rates, fall counts, and map quality metrics; store run manifests alongside map artifacts so regressions can be traced to configuration or calibration changes. Provide runbooks for degraded modes (switch to kinematic odometry + contact priors, reduce gait speed) and ensure the supervisor node can trigger these modes automatically when covariance or map health metrics cross thresholds.

### Example

Integrate ORB‑SLAM3 as a ROS 2 node with a Nav2‑based planner: convert fused depth into a traversability height image and occupancy grid consumed by a footstep planner. A supervisor node computes dynamic stability metrics and rejects unsafe plans; ros2_control executes validated trajectories. Gate CI merges with HIL rosbag2 replay tests that assert localization and safety metrics remain within thresholds.
