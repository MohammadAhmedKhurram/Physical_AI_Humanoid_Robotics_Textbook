Integration

Integration ensures that the robot description, simulation models, controllers, and perception pipelines remain coherent through the software lifecycle. This tab prescribes validation steps, CI integrations, and operational controls required for safe, repeatable robot deployments.

Integration steps
- Schema and linting: run URDF/SDF linters and schema checks as pre-merge gates.
- HIL smoke tests: spawn a headless Gazebo instance, load the robot model, and run a minimal control loop to verify joint limits and controller bindings.
- Asset verification: check mesh integrity, LOD suitability, and proper link-frame naming that maps to part numbers.

Operational traceability
Version every URDF/SDF with an associated manifest that lists controller versions, calibration artifacts, and test-suite hashes. Record ros2 bag traces for HIL runs and link them to the manifest so that any release can be reproduced end-to-end.

Physically grounded example
For a wheeled manipulator, integration requires confirming that URDF joint limits and transmission definitions match motor controller firmware. Run a CI job that loads the URDF into Gazebo, executes predefined joint motion profiles, and compares simulated torque/current traces against expected envelopes. If mismatches appear, the job flags the merge and provides diagnostics to engineering.

Acceptance checks
- [x] Validation steps and CI gates
- [x] Asset and manifest traceability
- [x] Physically grounded example included
