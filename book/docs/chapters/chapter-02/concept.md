Concept

Sensors are the primary conduit by which embodied agents perceive the physical world; perception is not an abstract software module but an instantiated, time‑sensitive mapping from transduced physical quantities to representations usable by planners and controllers. This chapter frames sensing as a systems problem: sensor selection, mounting and geometry, noise and latency budgets, and the epistemic limits that follow from physical measurement processes.

Modalities and their inductive biases
Each modality brings characteristic strengths and failure modes. Cameras provide dense spatial content but suffer from exposure, motion blur, and lighting-dependent failure modes; LiDAR yields accurate range geometry but is sensitive to surface reflectivity and angular resolution; IMUs offer high-rate inertial cues but accumulate bias over time; tactile arrays and force-torque sensors measure contact physics directly but are localized and sparse. Engineers should treat these properties as inductive biases that shape perception algorithms: depth completion networks leverage geometric priors, while visual odometry uses photometric invariance techniques to compensate for lighting.

Timing, synchronization, and the physical loop
Perception operates within tight timing constraints: sensing latency, processing delay, and actuator command periods compose an end-to-end control budget. ROS 2 middleware and hardware timestamping are essential tools for managing this budget. In fielded systems, the interplay between sensor sampling rates and mechanical dynamics determines whether a perception signal is usable for closed-loop control (for example, using a 200 Hz IMU for attitude stabilization vs a 10 Hz LiDAR for route planning).

Learning with sensors and sim-to-real considerations
Modern embodied learning pipelines rely on large-scale simulated data (Isaac Sim, Gazebo, Unity) and targeted domain randomization to handle distributional shifts. However, sensor modeling fidelity is critical: naively generated camera images or raycast LiDAR returns that ignore motion blur or multi-path reflectance produce brittle policies. Residual dynamics learning and online domain adaptation (e.g., adaptive sensor noise models tuned by ros2 bag replay and calibration) reduce catastrophic transfer failures.

Physically grounded example
An outdoor inspection UGV fitted with a 32-channel spinning LiDAR, stereo cameras, and an IMU illustrates these tradeoffs. The LiDAR provides reliable obstacle geometry for local planning; the stereo pair supports dense reconstruction useful for inspection tasks; the IMU provides high-rate orientation for controller stabilization. During rough-terrain trials the team observed LiDAR returns degrade in heavy dust, requiring sensor-fusion policies that dynamically reduce LiDAR reliance and increase stereo‑based depth estimation—an operational decision encoded in the autonomy stack.

Acceptance checks
- [ ] Explicit treatment of sensor modalities and biases
- [ ] ROS 2 and simulation tool references (Gazebo, Isaac, Unity)
- [ ] At least one physically grounded example
