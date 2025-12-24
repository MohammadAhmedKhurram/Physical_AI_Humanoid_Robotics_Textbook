System

A systems-oriented account of physical AI treats the robot as a distributed cyber-physical system comprising sensing, estimation, planning, control, and human–machine interfaces. This decomposition helps engineers allocate responsibilities and reason about latency, reliability, and safety budgets.

Architecture layers
- Perception: raw sensor drivers (ROS 2 nodes) publish time-stamped messages for cameras, LiDAR, IMUs, and tactile arrays. Sensor preprocessing includes message synchronization and denoising.
- State estimation: filters and factor-graph back-ends (e.g., GTSAM, Cartographer) fuse multimodal measurements to produce pose and map estimates with uncertainty.
- Planning: global planners provide waypoint sequences; reactive local planners (e.g., DWB, TEb) handle obstacle avoidance under latency constraints.
- Control: hierarchical controllers translate velocity or torque commands into actuator-level setpoints, using model-predictive or PID strategies.
- Orchestration and safety: system managers supervise lifecycle states, health monitoring, and fallback behaviors.

Interfaces and contracts
Specify API contracts as QoS settings, message types, and action servers. A motion planner action must document preconditions (map availability, localization confidence), outputs (timestamped trajectories), and failure modes (no path found). Define end-to-end latency budgets: sensor period, perception processing time, planner cycle, and actuator update rate.

Security and reliability
Architects must specify non-functional contracts: authentication for operator interfaces, resource quotas for CPUs/GPUs, and graceful overload behavior. Use QoS and deadline-aware scheduling to isolate perception and control cycles and define SLOs (e.g., perception latency p99 below X ms). Include rate limiting on high-bandwidth sensors so compute spikes do not starve control loops and watchdogs that transition the platform to a safe state when invariants fail.

Physically grounded example
Consider a warehouse forklift retrofitted with ROS 2: a 3D LiDAR provides obstacle point clouds, an IMU and wheel encoders supply odometry, and a CAN-connected motor controller executes velocity commands. The state estimation node runs Cartographer for mapping; a global planner computes aisles-to-goal routes while a local planner performs collision-free trajectory following. The system manager monitors CAN error frames and IMU anomalies; on detection, it triggers a slow-stop trajectory executed by the low-level controller to avoid tipping during emergency maneuvers. Operational constraints — maximum allowable tilt, center-of-mass shifts when lifting loads, and brake temperature limits — should be encoded so planner goals never violate mechanical safety envelopes.

Designing for robustness
Approaches include multi-sensor redundancy (camera + LiDAR), graceful degradation to conservative reactive controllers, and predictive monitoring (anomaly detection on sensor residuals). End-to-end verification includes HIL testing with Gazebo; telemetry from trials should feed back into simulation parameter updates and calibration.

Acceptance checks
- [x] System decomposition and interfaces
- [x] ROS 2 and real robot (forklift) example
- [x] No code dumps
