Concept

Foundations of Physical AI and embodied intelligence require reframing intelligence as situated, sensorimotor, and materially instantiated. Rather than treating cognition as abstract symbol manipulation, a physical-AI perspective foregrounds continuous perception–action loops, mechanical affordances, and constraints imposed by sensors and actuators. This tab synthesizes conceptual primitives—embodiment, situatedness, embodiment-dependent learning, and the role of physics—in a form useful to system architects and researchers.

Embodiment and situatedness
Embodiment posits that cognitive processes emerge through the coupling between a body and its environment. Sensors (e.g., LiDAR, IMUs, cameras) and actuators (e.g., differential drive, manipulator joints) impose delays, noise, and limitations that shape the feasible set of behaviors. Situatedness emphasizes that opportunities for action (affordances) are environment-specific: a narrow corridor affords different locomotion strategies than an open field. These constraints are not mere implementation details; they form the inductive biases exploited by embodied learning methods.

Perception–action loops and real tools
A practical engineering view centers on perception–action loops implemented on robotic middleware such as ROS 2. Sensors stream data to perception nodes which estimate state; planners produce motion commands that map to joint torques or wheel velocities; controllers close the loop, compensating for delays and unmodeled dynamics. Consider a ROS 2 mobile robot using a 2D LiDAR and wheel encoders: the SLAM node fuses odometry and scan data to provide a map-relative pose; a local planner generates velocity commands; a low-level PID controller on the motor driver tracks the velocity setpoints. Here, sensor latency and wheel slip limit achievable stability margins and controller gains.

Learning in physical systems
Learning on hardware must account for sample efficiency and safety. Sim-to-real transfer, domain randomization, and residual dynamics learning are common strategies. For instance, training a manipulation policy in Isaac Gym with randomized friction and mass properties and then fine-tuning on a real UR5 manipulator using force-torque sensing can substantially reduce catastrophic failures. The example underscores that physics-aware simulation and sensing choices materially impact learning outcomes.

Design implications
Architects must treat sensors and their failure modes as first-class citizens: sensing modality selection, calibration pipelines, and timing budgets shape algorithms and hardware choices. The conceptual lens advocated here is pragmatic: physical constraints guide model structure, learning curricula, and validation procedures. Designers should explicitly record the mapping from sensed quantities to control-relevant state variables and quantify uncertainty budgets.

Acceptance checks
- [x] Contains embodiment-focused primitives
- [x] Includes ROS 2 example (LiDAR + encoders) with constraints
- [x] No code dumps

