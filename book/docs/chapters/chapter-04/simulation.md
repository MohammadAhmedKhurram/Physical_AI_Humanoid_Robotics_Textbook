Simulation

Simulators rely on accurate robot descriptions to produce faithful dynamics and sensor interactions. This tab outlines model fidelity requirements, URDF/SDF best practices for Gazebo/Unity/Isaac, and recommended workflows for HIL and model calibration.

Model fidelity
Use appropriate levels of geometric and dynamic fidelity: reduce visual mesh complexity for runtime performance but preserve collision geometry fidelity for accurate contact dynamics. Specify accurate inertial parameters and joint friction models; verify damping and stiffness parameters in the simulator against measured step responses.

URDF and SDF practices
- URDF: include collision and visual meshes, inertial tags, and transmission elements. Use xacro for parameterized robot descriptions and maintain a clear separation between robot geometry and environment scenes.
- SDF: leverage nested models, link joints with multi-DOF definitions, and embed sensor and plugin configurations for Gazebo/Ignition.

Calibration and system identification
Perform system identification by running torque/position sweeps and comparing measured joint responses (torque, current, encoder) against simulation. Automate parameter tuning by minimizing residuals between simulated and measured trajectories.

Physically grounded example
To calibrate a quadruped in Gazebo, use URDF with realistic collision meshes and measured mass/inertia. Run joint torque step tests on the real robot while replaying the same commands in Gazebo; use optimization to tune inertial parameters and joint friction until simulated motor current traces align with hardware measurements.

Acceptance checks
- [ ] Model fidelity and tool references (Gazebo, Isaac, Unity)
- [ ] Calibration and system identification example
- [ ] No code dumps
