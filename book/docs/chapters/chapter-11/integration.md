Integration — Deploying Conversational Robots in Real Environments

Integration checklist
- Hardware validation: verify microphone array calibration, camera intrinsics/extrinsics, and actuator limits on target robot platform.
- Network & compute: configure hybrid compute (on‑device for ASR/VAD, edge/cloud for heavy NLU) with robust fallback. Ensure TLS and authentication for all remote model endpoints.
- Data flows: instrument telemetry for ASR errors, NLU confidence, dialog state transitions, and task outcomes. Use ROS 2 diagnostics and a centralized observability stack (Prometheus/Grafana) for SLO monitoring.

Deployment patterns
- Edge-first: run low-latency services locally and optionally offload complex NLU. Use model pruning and quantization on Jetson-class devices.
- Cloud-assisted: allow cloud models for advanced language understanding but require a fallback policy when connectivity degrades (explicit user notification and local simplified NLU).

Physically grounded example (hospital deployment)
- Pre-deployment: run integration tests in a replica simulation (lobby model) including noisy audio scenarios, occlusions, and multiple concurrent speakers.
- Deployment: roll out to a single floor with monitoring enabled. Validate that ASR and NLU confidence distributions match simulation expectations. If a drop is detected, throttle cloud offload and switch to conservative policies (more clarifying questions, reduced autonomy).

Safety and compliance
- Privacy: log only metadata (intent, confidence) unless the user opts into audio/video retention. Encrypt stored logs at rest and in transit.
- Accessibility: provide alternative interaction modes (touchscreen UI, visual prompts) for users with speech impairments.

Integration acceptance tests
- Real-world speech under SNR >= 10 dB: ASR WER < threshold, intent accuracy within acceptable bounds
- Human-in-the-loop safety: emergency stop triggered and validated within spec
- Continuous monitoring: alerts for drift in ASR/NLU metrics

Conclusion
Integration requires careful coordination across hardware, networking, and operational monitoring. The hybrid compute model with strong local fallbacks provides the most robust balance between capability and privacy.