System

Robot description formats provide canonical interfaces between modeling, simulation, control, and perception. This tab specifies system-level contracts for URDF/SDF artifacts, tooling, and verification processes needed to ensure consistency across the software stack.

Artifact contracts
- Geometry and collision semantics: provide both visual and collision meshes and specify level-of-detail (LOD) for simulation vs on-board visualization.
- Transmission and controller bindings: document how joint commands map to motor drivers (position, velocity, torque control) and specify control interface expectations.
- Sensor mounts and frames: define exact TF frames, sensor intrinsics, and extrinsic transforms to avoid misalignment between perception and control.

Verification and CI
Embed schema checks and URDF/SDF linting into CI pipelines. Use XSD or schema validators where available and write unit tests that spawn a headless Gazebo instance importing the URDF/SDF to validate joint limits, self-collision checks, and successful controller loading.

Versioning and traceability
Associate each URDF/SDF revision with a manifest that includes asset versions, controller versions, and calibration artifacts. Maintain mapping between URDF links and physical part numbers so replacement parts do not invalidate simulation assumptions.

Physically grounded example
For a legged robot, provide multiple collision mesh resolutions: a coarse convex hull for plan-time collision checking and a higher-fidelity mesh for HIL simulation in Gazebo. Document how joint limits map to safety envelopes used by the low-level controller; when replacing a footpad part, update the manifest and run self-collision and gait tests to confirm continued safety.

Acceptance checks
- [x] Artifact contracts and verification described
- [x] Versioning and traceability
- [x] Physically grounded example included
