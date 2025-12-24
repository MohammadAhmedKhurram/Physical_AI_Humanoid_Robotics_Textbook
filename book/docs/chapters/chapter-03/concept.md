Concept

Concept

ROS 2 functions as the robotic nervous system: a middleware layer that connects perception, planning, control, and human interfaces into coherent runtime graphs. This chapter provides a conceptual overview of ROS 2’s role in distributed robotics, emphasizing its DDS-based transport, lifecycle management, QoS semantics, and its fit within high-assurance development practices.

Middleware as architecture
ROS 2 abstracts middleware concerns so developers can reason about distributed nodes and composition rather than low-level sockets. DDS provides discovery, reliable/unreliable transport modes, and fine-grained Quality of Service (QoS) controls: deadline, liveliness, durability, and reliability. Treating these primitives as architectural knobs is essential when designing systems with mixed-criticality requirements (e.g., safety-critical controllers vs high-bandwidth perception streams).

Lifecycle and composability
The ROS 2 node lifecycle standardizes startup, configuration, activation, and cleanup phases, enabling deterministic orchestration and safe reconfiguration. Composed nodes allow colocating multiple components in a single process to reduce serialization overhead and enable intra-process communication for low latency—an important consideration for control loops.

Time, clocks, and determinism
Robotic systems require consistent time bases. ROS 2 supports hardware timestamping and leveraging PTP/NTP synchronization; simulation-integrated time (e.g., Gazebo simulated time) simplifies reproducible testing. Determinism depends on careful QoS tuning, real-time kernels, and avoiding non-deterministic allocations in hot paths. To achieve predictable behavior, define explicit deadline and liveliness budgets per-topic and enforce them in lifecycle checks. Additionally, enforce deterministic resource allocation policies and avoid dynamic memory operations in control hot paths.

Physically grounded example
A warehouse mobile manipulator uses ROS 2 to integrate a high-rate IMU, a 32-channel LiDAR, and an arm controller. QoS settings assign RELIABLE delivery and lower deadline for the arm controller, while LiDAR uses BEST_EFFORT with burst buffering for point clouds. Lifecycle management ensures that the arm controller only activates after localization confidence exceeds a threshold; composed nodes on the embedded computer reduce latency between controller and perception. Operators validate QoS and lifecycle behavior during HIL trials using Gazebo and ros2 bag traces; they also run latency and throughput sweeps against the system under loaded conditions.

Acceptance checks
- [x] DDS and QoS concepts explained
- [x] Lifecycle, composability, and timing discussed
- [x] Physically grounded ROS 2 example included
