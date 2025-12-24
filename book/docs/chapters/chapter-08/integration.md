Integration

Integration focuses on bringing the full stack together, validating cross-cutting behaviors, and establishing safety and deployment procedures. Start with continuous integration pipelines that run static checks on URDF/SDF, validate message interfaces, and run lightweight simulation smoke tests (headless Gazebo or Isaac Sim). Use ROS 2 bag playback as a canonical data source for regression testing: run the state estimator, planner, and controller on recorded sensor streams and verify logs for ZMP and CoM tracking errors.

Operational integration requires defining deployment artifacts: containerized ROS 2 nodes (Docker images), launch descriptions, and configuration profiles (real hardware vs. simulation). A recommended structure: image tags for each subsystem (estimation:v1, planning:v1, control:v1), a deterministic deployment manifest, and versioned URDF/SDF in git. Integration tests should include HIL checks where the controller drives torque commands against a simulated actuator model. A physically grounded example: perform a staged integration where first the estimator runs against VICON-replayed bag files to validate CoM estimate stability; next, the planner/controller loop runs in Gazebo, and finally the same containerized components are deployed on hardware in a tethered test.

Monitoring and safety: set up Prometheus exporters for key metrics (CoM error, ZMP margin, CPU latency), visualize in Grafana dashboards, and configure alerting rules for safety thresholds (ZMP margin < 0.02 m, joint torque near limit). Implement state machine fallbacks: limp mode (reduce torque), sit sequence, or motor cutoff. Document runbooks for emergency stop and recovery procedures.

Acceptance criteria:
- [ ] CI runs URDF validation and headless simulation smoke tests
- [ ] Containerized deployment works both in simulation and hardware (same images)
- [ ] Monitoring exports and alerts configured
- [ ] Runbooks documented in ops/README.md

Next steps: calibrate actuator models against bench tests, expand simulation scenarios, and consider ADR if changing middleware (e.g., moving from ROS 2 DDS vendor to a different RT middleware).

Code references: launch/walking_stack.launch.py:12, ops/README.md:4