System

Designing ROS 2 systems requires explicit contracts for node responsibilities, inter-process communication, and resource allocation. This tab lays out architectural patterns for middleware deployment in distributed robotic fleets and single-vehicle systems alike.

Architectural patterns
- Single-vehicle, composed-process pattern: colocate latency-sensitive perception and control nodes in a single process using ROS 2 composition to minimize latency and use intra-process transport.
- Distributed fleet pattern: run per-vehicle gateways that aggregate telemetry and provide secure operator interfaces. Use DDS security to authenticate and encrypt inter-vehicle communications.
- HIL and staging pattern: run the same ROS 2 graph in simulated time for HIL tests so node lifecycles and QoS behave identically in simulation and hardware.

Interface contracts
Define message schemas, action semantics, and QoS policies. For a controller command topic, explicitly document reliability, deadline, and history depth. Use ROS 2 IDL/msg for type safety and include invariants (command ranges, safety preconditions) in API documentation. Maintain a contract registry and automated schema checks as part of CI to detect breaking changes prior to deployment.

Monitoring, health, and resource governance
Implement lifecycle-driven health checks and export Prometheus metrics for node latency and queue sizes. Use resource governance (cgroups, CPU pinning) and deadline-aware scheduling to ensure control loops are protected from perception spikes. Instrument DDS metrics to monitor discovery times and message latencies and surface alerts when drift occurs. Additionally, implement a watchdog manager that sequences deterministic restarts and escalates to safe-stop behaviors when critical nodes fail repeatedly.

Physically grounded example
A ROS 2-based fleet of inspection UGVs uses per-vehicle gateways to report summarized telemetry to an operator station. Each vehicle composes perception and local planning in a single process while mission orchestration runs separately. DDS security enforces mutual authentication; QoS tuning ensures critical command topics are RELIABLE with small history depth while high-bandwidth sensor topics are BEST_EFFORT with larger queues. Fleet gateways perform certificate rotation and telemetry aggregation for operator dashboards. During a field trial, one vehicle experienced an unexpected CAN bus noise pattern; telemetry analysis via the gateway allowed remote diagnosis and triggered a safe‑stop until a maintenance action was taken.

Design considerations and trade‑offs
Choosing composition reduces IPC overhead but can increase blast radius of process failures; prefer composition for tightly coupled subsystems and separate processes for mission-critical isolation. QoS tuning balances reliability against latency and resource usage; measure trade‑offs empirically during HIL sweeps. Security measures (DDS security, PKI) add operational complexity but are required for fleet deployments in untrusted networks.

Acceptance checks
- [x] Architectural patterns and QoS contracts
- [x] Monitoring and resource governance discussed
- [x] Physically grounded example included
- [x] Design trade-offs noted


