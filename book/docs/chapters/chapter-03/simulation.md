Simulation

ROS 2 integration with simulators is central to reproducible testing and HIL validation. This tab describes best practices for connecting ROS 2 with Gazebo, Unity, and NVIDIA Isaac, and for ensuring simulated time, topics, and lifecycles map to hardware behavior.

Simulator integrations
- Gazebo: native ROS 2 plugins and ros2_control integration make Gazebo suitable for driver-level HIL testing and middleware verification. Use headless mode for CI regression runs and connect real drivers via bridged topics for partial HIL.
- Unity: with ROS-TCP-Connector and ROS 2 packages, Unity supports high-fidelity rendering and C#-based sensor plugins that produce photorealistic data for vision pipelines and synthetic dataset generation.
- NVIDIA Isaac: Isaac Sim integrates directly with ROS 2 and provides GPU-accelerated physics and sensor synthesis for large-scale training workloads, enabling massively parallel data production for learning-based perception.

Time and lifecycle alignment
Ensure simulators publish simulated time and ROS 2 nodes respect /use_sim_time; validate that node lifecycles behave identically when running in simulated time. Use deterministic seeds and record simulation manifests (asset versions, physics parameters) and randomization seeds to ensure reproducibility. Connect CI runners to artifact storage where manifests and ros2 bag outputs are archived.

Physically grounded example
Connect a Gazebo world containing a warehouse layout to a ROS 2 graph that includes Cartographer and a local planner. Run headless Gazebo in CI to simulate obstacle injection scenarios and validate planner failure modes. For vision pipelines, use Unity or Isaac to generate diverse lighting and material conditions and record ros2 bag datasets for model training. Partial HIL tests can bridge a real sensor driver into a simulated sensor stream to validate middleware behavior before hardware trials.

Acceptance checks
- [x] Gazebo, Unity, Isaac ROS 2 integrations described
- [x] Simulated time and lifecycle mapping discussed
- [x] Physically grounded example included


