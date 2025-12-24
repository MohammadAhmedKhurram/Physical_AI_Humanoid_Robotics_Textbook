---
id: chapter-09
title: "Chapter 09 — Manipulation & Grasping"
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

Manipulation and grasping are the study of how robotic hands and whole bodies perceive, plan, and execute contact‑rich interactions with objects to accomplish tasks. Core theoretical primitives include grasp taxonomy (power, precision, pinch), contact kinematics and contact mechanics (point, line, patch contacts), wrench‑space reasoning for force closure, and the interplay between kinematic reachability and dynamic balance for humanoids. For humanoid platforms, manipulation is not an isolated arm problem: reachability, center‑of‑mass (CoM) margins, and foot placement interact with end‑effector tasks, so planners often formulate prioritized objectives that trade off manipulation quality against stability metrics.

Perception converts sensor streams (RGB‑D, stereo, tactile arrays) into affordance representations and grasp candidates. Classical pipelines pair point‑cloud segmentation (PCL) and primitive fitting with analytic grasp synthesis (antipodal grasps, wrench space checks), while modern data‑driven approaches (DexNet, GG‑CNN, end‑to‑end affordance networks) predict grasp scores from RGB‑D patches. Control strategies range from open‑loop motion primitives suitable for structured settings to compliant, impedance‑based controllers that modulate contact forces during unexpected interactions. Slip detection and force‑based adaptation are central if the system must maintain robust grasps in cluttered environments.

Grasp quality and task success are evaluated with measurable metrics: force‑closure probability, required grasp wrench space margin, probability of slip under measured perturbations, and end‑to‑end task success rate under randomized poses. Robust manipulation stacks embed reactivity—grasp refinement, regrasping, and active perception (viewpoint adjustment) are common recovery strategies when candidate grasps present low confidence.

Physically grounded example: perform a tabletop pick experiment with a humanoid arm equipped with a Robotiq 2F‑85 gripper and an Intel RealSense D435i mounted on the head. Use a GG‑CNN variant running on a Jetson AGX Xavier to generate grasp affordance heatmaps; feed top candidates into a MoveIt 2 pipeline for whole‑body inverse kinematics and collision‑aware approach trajectories. Execute grasps under wrist F/T monitoring and classify success by slip incidence and hold time over 5 s. Acceptance checks: grasp success ≥ 90% on a curated object set, mean approach time, and slip rate.

</TabItem>

<TabItem value="system" label="System">

A manipulation system integrates hardware, sensing, middleware and control with explicit real‑time boundaries and safety envelopes. Hardware choices—underactuated adaptive hands versus multi‑finger dexterous hands, wrist F/T sensors, tactile skins, and compliant wrists—set the envelope of achievable contact behaviours. Sensors should provide dense spatial information (RGB‑D or stereo) and fast contact feedback (wrist F/T and tactile arrays sampled at ≥1 kHz where available) so that the control stack can detect contact onset and slipping rapidly.

Middleware conventions (ROS 2, ros2_control, controller_manager) provide deterministic interfaces for controllers and actuators. Motion planning commonly uses MoveIt 2 for collision‑aware path generation while whole‑body optimization libraries (Crocoddyl, HQP frameworks) handle prioritized tasks and torque allocation. For real‑time loops, prefer a deterministic torque/effort interface with control update rates matched to actuator capability (torque loops 500–1,000 Hz where possible; admittance/impedance layers at 200–500 Hz). A supervisory layer monitors estimator covariance, torque margins and slip detectors and triggers safe fallbacks.

Architectural invariants include explicit contracts for latency (perception-to-actuation budget), bounded solver iteration budgets for optimization, and safety checks (force thresholds, joint limit margins). Partition compute by function: Jetson/AGX for perception and learned models, NUC/PC for planning and optimization, and an isolated RT core for low‑latency control where PREEMPT_RT is available. QoS and DDS tuning in ROS 2 is necessary to prioritize sensor and command topics under constrained networks.

Physically grounded example: on an HRP‑style humanoid testbed, run perception on a Jetson AGX Xavier (RGB‑D segmentation + affordance network), planning on an Intel NUC (MoveIt 2 + Crocoddyl), and a real‑time torque controller on an RT core interfacing via EtherCAT. The supervisor subscribes to wrist F/T at 1 kHz and triggers a safe‑hold if average contact force exceeds configured thresholds or if CoM margins reduce below safe limits during an approach.

</TabItem>

<TabItem value="simulation" label="Simulation">

Simulation is central to validating contact interactions, perception training and controller stability prior to hardware trials. Use Gazebo/Ignition or MuJoCo for physics‑accurate contact dynamics and HIL‑style control loops; use NVIDIA Isaac Sim or Unity for photoreal rendering of sensor inputs to train and stress perception models. Accurate contact parameterization (friction coefficients, contact stiffness, damping) is essential—to meaningfully reproduce slip behaviour and contact transients, tune simulator contact models against bench measurements (force sensors, contact tests).

For learned grasping, large synthetic datasets are generated in Isaac Sim or Unity with randomized object meshes, textures, lighting and clutter. Export synchronized RGB‑D, normals and affordance labels to train networks (DexNet‑style or GG‑CNN). Domain randomization should be physically grounded: randomize mass, friction, sensor noise and latency within empirically measured bounds to avoid unrealistic regimes that harm transferability.

HIL benches combine a real motor driver or torque interface with a simulated contact environment to validate timing and saturation behaviours. Deterministic scenarization (seeded terrain and object placements) with rosbag2 manifests supports reproducible CI tests. Metrics to validate in simulation include grasp success under disturbances, slip incidence under measured perturbations, and control loop timing stability.

Physically grounded example: render 20,000 cluttered tabletop scenes in Isaac Sim and train an affordance network. Validate the network in Gazebo HIL with a URDF matched to bench‑measured inertias and contact geometry; run a Monte Carlo sweep across friction and sensor delay parameters and assert grasp success and slip rates remain within acceptance bands before advancing to hardware.

</TabItem>

<TabItem value="implementation" label="Implementation">

Implementation follows a staged, test‑driven pipeline: (1) create authoritative digital artifacts (accurate URDF/SDF, measured inertias, hand meshes and contact geometry); (2) collect calibration artifacts (hand‑eye calibration, wrist F/T offsets); (3) prototype perception and analytic graspers for baseline validation; (4) integrate learned affordance models and whole‑body planners; (5) escalate through HIL and guarded hardware trials.

Driver and middleware best practices include using hardware timestamps, exposing camera_info and F/T calibration topics, and providing deterministic ros2_control controllers with lifecycle management. Start with analytic grasp synthesis (antipodal, wrench‑space checks) to validate kinematics and collision handling, then integrate learned affordances (GG‑CNN, DexNet variants) for clutter and novel objects. For control, adopt an admittance or impedance layer between planned trajectories and torque commands to absorb unmodelled contacts; use QP solvers (OSQP, qpOASES) or Crocoddyl for prioritized whole‑body allocation.

Testing scaffolds include deterministic rosbag2 regression fixtures, parameter sweep harnesses for control thresholds, and CI jobs that replay simulation scenarios and assert solver convergence, grasp success and slip metrics. HIL benches should run motor drivers in current‑limit modes and record rosbag2 while exercising grasp sequences with randomized perturbations.

Physically grounded example: implement a pipeline where a GG‑CNN grasp predictor runs on a Jetson AGX Xavier to supply grasp candidates; MoveIt 2 generates approach paths; an admittance controller with wrist F/T feedback executes the closure and monitors slip. Validate across 100 household items with randomized poses; acceptance criteria: ≥92% success on the curated set, slip rate < 5% under nominal perturbations.

</TabItem>

<TabItem value="integration" label="Integration">

Integration operationalizes manipulation into task‑level autonomy and production workflows. Define versioned IDLs for grasp descriptors, pick/place intents and outcome provenance metadata that include sensor manifests, candidate scores and calibration checksums. Supervisory logic must evaluate grasp confidence, balance margins and torque margins and choose recovery policies (regrasp, nudging, operator intervention) when thresholds are violated.

Operational practices include artifact registries for grasp policies and datasets, CI gates that run deterministic simulation suites, and telemetry dashboards that surface slip rates, grasp latencies and failure modes. Maintain runbooks for degraded modes (safe‑hold, conservative impedance) and ensure HIL acceptance gates validate torque margins and controller stability under current‑limit conditions.

For auditability and reproducibility, store rosbag2 artifacts, policy versions, URDF/calibration checksums and manifest metadata for every promoted build. This enables traceable regression reports when grasp distributions or failure modes shift after policy updates.

Physically grounded example: integrate a manipulation stack where a perception node publishes affordance heatmaps to a task manager; when grasp confidence is high the task manager sequences approach, grasp and place behaviours. If grasp confidence is low, execute a regrasp policy or request supervised intervention. Record the full rosbag2 manifest and attach it to the CI job that gated the release.

</TabItem>

</Tabs>
