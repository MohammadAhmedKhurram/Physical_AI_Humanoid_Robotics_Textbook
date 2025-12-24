Implementation — Building Conversational Robot Components

Component roadmap
1. Audio front-end: integrate a microphone-array driver (USB or I2S) with ROS 2 audio_capture node. Implement beamforming as a node or use a library (BeamformIt, Pyroomacoustics). Add VAD to gate ASR requests.
2. ASR & NLU: choose a streaming-capable ASR (ESPnet streaming, Kaldi, or cloud streaming). Provide on-device fallback using a smaller acoustic model. NLU should be modular: intent classification, slot filling, and a semantic lexicon to map phrases to known entities.
3. Semantic mapping: subscribe to perception topics (object detections, tracked people); implement a resolver that matches NLU slots to physical entities using proximity, name maps, and temporal continuity.
4. Dialog manager & policy: finite‑state or neural policy that supports clarification, repair, and multi-step tasks. Expose actions as ROS 2 services.
5. Planning & control: use Navigation2 for base navigation; integrate manipulation stacks (MoveIt2) for grasping. Ensure planners respect safety_monitor signals.

Safety and testing
- Safety monitor must be independent and high-priority, able to preempt motion commands. Implement a human proximity detector using depth sensors.
- Unit test nodes (mock sensors) and integration tests (simulated Gazebo world) to exercise full pipelines.

Physically grounded example (implementation checklist)
- Set up ROS 2 node graph: audio_capture -> beamformer -> streaming_asr -> nlu -> dialog_manager -> planner -> safety_monitor.
- Implement an acceptance test: user says "Bring my water bottle" in simulation; the system must resolve referent, plan a route, grasp the bottle, and deliver while obeying safety stops.

Acceptance criteria (example)
- End-to-end success rate ≥ 90% on scripted tasks in simulation
- ASR streaming latency median < 300 ms
- Dialog manager issues clarifying prompt in ambiguous referent cases > 95% of the time instead of attempting action

Implementation notes
- Avoid sending raw audio to the cloud without user consent; anonymize or obfuscate sensitive audio. Prefer local ASR for private settings.
- Keep NLU models updateable through a defined model registry and versioning strategy (semantic break detection on updates).