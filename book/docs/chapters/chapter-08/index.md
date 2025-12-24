---
id: chapter-08
title: "Chapter 08 — Humanoid Locomotion & Balance"
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs
  defaultValue="concept"
  values={[
    {label: 'Concept', value: 'concept'},
    {label: 'System', value: 'system'},
    {label: 'Simulation', value: 'simulation'},
    {label: 'Implementation', value: 'implementation'},
    {label: 'Integration', value: 'integration'},
  ]}
>

<TabItem value="concept" label="Concept">

Humanoid locomotion and balance integrate classical mechanics, state estimation, and control theory to enable repeatable bipedal behaviour across uncertain, contact‑rich environments. Foundational primitives are the robot’s center of mass (CoM) relative to its base of support (BoS), the Zero Moment Point (ZMP) useful for quasistatic analyses, the Center of Pressure (CoP) that summarises foot contact forces, and dynamic stability measures such as the Capture Point and Divergent Component of Motion (DCM) that emerge from linear inverted pendulum approximations (LIPM). Bipedal locomotion is hybrid: alternating single‑support and double‑support phases punctuated by impacts. Designing robust locomotion therefore separates (1) high‑level footstep placement and mode selection, (2) mid‑level predictive trajectory generation (MPC/preview controllers) and (3) low‑level stabilizing controllers (ankle/hip torque modulation or whole‑body QPs).

Theoretical trade‑offs are explicit. Longer predictive horizons in MPC increase disturbance tolerance but require greater compute and more accurate models; ZMP/preview controllers are simpler and energy‑efficient but conservative for dynamic maneuvers; torque‑level whole‑body control offers compliance and better disturbance rejection but demands low‑latency hardware and reliable state estimates. Sensors constrain achievable bandwidth: a 200–1,000 Hz IMU and sub‑millisecond joint encoders enable tight inner loops, while foot force/torque sensors and pressure arrays provide contact quality and support region information used to tighten constraints in planners.

Recovery strategies form a minimal taxonomy: adjust ankle torques to modulate CoP (ankle strategy), exploit hip torques and swing leg inertia (hip strategy), and take an adaptive capture step (step strategy). A robust stack pairs model‑based planners with perception and parameter adaptation: learned foothold scorers from RGB‑D pipelines can bias footstep priors, while online identification (mass/CoM drift) refines predictive models.

Physically grounded example: perform a lateral push‑recovery experiment in Gazebo using a ROS 2 robot description (URDF) instrumented with an onboard IMU (200 Hz) and ankle F/T sensors. Fuse IMU and joint encoders with robot_localization to estimate base pose and CoM rate, compute the DCM, and run an LIPM‑based MPC that proposes adjusted footstep targets. Success metric: proportion of lateral impulse trials (e.g., 5 N·s) in which the controller restores the CoM within the updated BoS within two corrective steps.

</TabItem>

<TabItem value="system" label="System">

A dependable locomotion system stacks hardware, middleware, estimation, planning and control with clearly defined real‑time boundaries and safety envelopes. Hardware choices (series‑elastic vs direct‑drive actuators, ankle F/T vs pressure arrays, tendon routing) set achievable control bandwidths and sensing modalities. Middleware conventions favour ROS 2 for message routing, ros2_control for deterministic actuator interfaces, and controller_manager for lifecycle orchestration. Real‑time constraints are explicit: torque loops run at 500–1,000 Hz where actuators permit; state estimation and mid‑level planners inhabit the 100–200 Hz band; perception (height‑maps, segmentation) typically runs at 10–60 Hz and is partitioned onto GPUs.

State estimation should be multi‑rate and robust to outliers: preintegrated IMU factors (GTSAM), UKF/EKF stacks or factor‑graph solutions reconcile encoder kinematics, IMU, and F/T signals to publish low‑latency CoM/pose estimates with covariance. Rigid‑body libraries (Pinocchio, RBDL) provide dynamics primitives (mass matrix, Coriolis) for inverse‑dynamics controllers. Networked DDS QoS settings must prioritise IMU and command topics while allowing lower QoS for bulk perception frames.

Control architecture is driven by actuator capability and safety requirements. A conservative stack uses position/velocity controllers with timed footsteps and ZMP checks; a high‑performance stack uses an MPC planner over a reduced‑order model and a hierarchical whole‑body QP that enforces contact inequalities and torque limits. Admittance/impedance layers between planner and motor drivers provide compliance for unmodelled contacts. Supervisory logic monitors solver iteration counts, torque margins, estimator covariance and triggers fallback behaviours (reduce speed, conservative gait) when thresholds are crossed.

Operational practices include CPU/GPU partitioning (e.g., Jetson for perception, Intel NUC for planners, isolated RT core for torques), affinity and PREEMPT_RT scheduling for critical threads, and artifact manifests for URDF/calibration checksums. Instrumentation should surface latency histograms, control loop jitter, and energy per step to dashboards for regression detection. Maintain a documented safety chain: hardware E‑stop, software e‑stop, controller watchdogs, and controller envelopes that limit step length, velocity and commanded torques.

Physically grounded example: on an EtherCAT‑driven humanoid testbed, run ros2_control on a PREEMPT_RT kernel with a dedicated RT core. An ATI Mini40 ankle F/T provides contact feedback; the hardware_interface publishes joint states at 1 kHz, robot_localization fuses sensors at 200 Hz, and a whole‑body QP controller computes torque commands at 500 Hz. Perception (foothold classification) runs on an onboard GPU using NVIDIA Isaac components; DDS QoS and watchdogs enforce safe degraded modes if estimator latency exceeds a threshold. Use ros2_tracing and observability dashboards to ensure latencies and solver iterations meet acceptance criteria.

</TabItem>

<TabItem value="simulation" label="Simulation">

Simulation is indispensable for iterative design, randomized stress testing and the systematic reduction of risk prior to hardware trials. Use paired simulator modalities: physics‑accurate engines (Gazebo/Ignition, Drake, MuJoCo) for contact dynamics and control‑in‑the‑loop verification, and photoreal or GPU‑accelerated platforms (NVIDIA Isaac Sim, Unity) for perception realism and dataset generation. Critical simulator concerns are contact model fidelity (penalty vs constraint solvers, friction cone discretisation), solver stability (time‑step, constraint iterations), and actuator realism (torque limits, motor dynamics, series elasticity). Sensor models must capture bias growth, delay, quantization and rolling‑shutter effects when relevant.

Calibration and identification benches close the sim‑to‑real gap: measure contact compliance and friction on hardware, fit compliant contact models in the simulator, and validate by replaying standardized actuator commands while comparing torque/velocity traces. Deterministic scenarization (seeded terrain generators, scripted pushes) supports reproducible Monte‑Carlo trials; record rosbag2 artifacts with manifests and seeds so CI pipelines can replay identical runs.

Domain randomization is powerful but must be physically grounded: randomise mass, CoM offsets, friction, sensor noise and latency within experimentally measured bounds. Excessive randomisation produces unrealistic regimes that reduce transferability; instead, progressively widen randomisation ranges after hardware validation. For perception, augment photoreal frames from Isaac Sim with calibrated noise models and validate learned foothold scorers by replaying frames to the live perception stack.

Automation and metrics: run containerized batch jobs that sweep friction, payload and solver parameters and compute recovery metrics (fall incidence, recovery time, energy per meter, CoP/DCM residuals). Use bootstrap confidence intervals to compare controllers statistically and enforce CI gates based on defined thresholds. HIL benches that connect real motor drivers or torque sensors to simulators validate control loops under realistic latency and saturation; these benches should be part of the acceptance pipeline before any unsupervised hardware trials.

Physically grounded example: build a Gazebo environment with mixed terrain (flat sections, slabs, compliant mats) and a scripted lateral push rig. Run 1,000 episodes with randomized friction and payload; record rosbag2 and compare capture‑point recovery statistics against Isaac Sim experiments. Tune solver iterations and time step until recovery statistics converge between simulators and bench measurements, then promote the controller to HIL testing.

</TabItem>

<TabItem value="implementation" label="Implementation">

Implementation follows a staged, measurement‑driven pipeline that minimizes hardware risk and ensures reproducibility. Start by establishing authoritative digital artifacts: an accurate URDF/SDF with measured inertial properties and validated contact geometry, a manifest of calibration outputs (Kalibr camera/IMU results), and standardized rosbag2 sequences for regression. System identification captures joint friction, gearbox compliance, motor constants and contact compliance; these parameters are used to constrain model predictive controllers and condition solver numerical stability.

Driver and middleware best practices matter: prefer hardware timestamps, expose camera_info topics, and implement a synchronization layer when hardware timing is unavailable. Use ros2_control and controller_manager for deterministic actuator interfaces; prefer EtherCAT or similarly real‑time fieldbuses where available. Partition compute across devices (Jetson for perception, NUC for planning, isolated real‑time core for low‑latency torque loops) and enforce CPU affinity and PREEMPT_RT scheduling for critical threads.

Controller development is iterative: prototype with ZMP/preview and timed footsteps to validate kinematics and contact sequencing, then migrate to predictive MPC over a reduced‑order model (LIPM or variable‑height LIP) for dynamic stepping. Pair the planner with a hierarchical whole‑body QP for torque allocation that respects friction cones and actuator limits. Use Pinocchio for dynamics primitives and Crocoddyl or HQP frameworks for trajectory and inverse dynamics; adopt OSQP or qpOASES for real‑time QP solving and tune solver warm‑starting and iteration budgets to meet latency constraints.

Testing scaffolds are essential: automated rosbag2 replay tests with ground truth comparisons (ATE/RPE), parameter sweep harnesses for thresholds (feature matching ratio, keyframe insertion interval), and CI jobs that replay standardized scenarios. HIL benches should run motor drivers in current‑limit or torque‑limit safe modes, recording rosbag2 and solver diagnostics. Define acceptance criteria (e.g., estimation latency <100 ms, capture‑point recovery in ≤2 corrective steps at given impulse levels) and gate promotions to HIL/hardware on these metrics.

Physically grounded example: implement a two‑stage controller where a ROS 2 node runs an OSQP‑backed MPC on the LIPM at 50 Hz to propose footsteps and CoM references; a real‑time whole‑body torque controller (500 Hz) enforces those references using inverse dynamics computed by Pinocchio. Validate in Gazebo with domain randomization, escalate to HIL with motor drivers in current‑limit mode, and only then execute supervised low‑speed hardware trials while recording rosbag2 for every run.

</TabItem>

<TabItem value="integration" label="Integration">

Integration operationalizes locomotion for safe, repeatable deployment and developer feedback. Define precise, versioned data contracts (ROS 2 message/IDL definitions) for footstep plans, CoM/CP/ZMP trajectories, sensor health diagnostics, and whole‑body setpoints that include covariance and provenance metadata. Integration must balance coupling and isolation: loose coupling (SLAM publishes maps and pose; planner queries snapshots) simplifies failure isolation, while tighter coupling (planner constraints embedded within SLAM optimization) can reduce conservatism for complex maneuvers though it increases verification complexity.

Supervisor and safety layers enforce runtime decisioning. Supervisory logic consumes estimator covariance, torque margins, and environmental risk metrics (slippery ground probability, foothold confidence) to accept, adapt or reject plans. Safety architecture is layered: hardware E‑stop, software e‑stop, controller watchdogs, envelope constraints on step length/velocity/torque, and conservative fallback gaits for degraded perception or estimator health. Implement atomic map update semantics and map registries with checksums to avoid partial writes observed by planners.

Operational observability and CI are critical. Export metrics (estimation ATE/RPE, control latency, solver iterations, energy per step, fall counts) to dashboards and link run artifacts (rosbag2, solver logs, manifests) for post‑mortem analysis. CI gating should execute deterministic rosbag2 replays and assert recovery and safety metrics; HIL gates must verify torque margins and controller stability under current‑limit conditions before any unsupervised hardware promotion.

Artifact and release management matter: store URDF/calibration checksums, launch bundles, and validated parameter sets in an artifact registry. Maintain retraceable experiment manifests for every controller release that include simulator versions (Gazebo/Isaac), random seeds, and hardware bench calibration. Provide runbooks with abort criteria and fallback modes for operators.

Physically grounded example: integrate an Isaac Sim perception pipeline that supplies a foothold costmap to a ROS 2 planner. Execute a staged sequence: (1) closed‑loop simulator rehearsals, (2) HIL with motor drivers in current‑limit mode and telemetry recording, and (3) supervised low‑speed hardware trials. At each stage, perform rosbag2 comparisons of perception signatures and verify supervisor triggers (fallback to conservative gait) within defined thresholds; only promote builds that satisfy CI and HIL acceptance criteria.

Ensure experiment manifests, telemetry, and runbooks are archived with each release to support audits and traceability.

</TabItem>

</Tabs>
