---
title: "Chapter 10 — Vision-Language-Action (VLA) Systems: Implementation"
tab: Implementation
word_count: 390
---

Implementation of VLA systems stitches together reproducible pipelines for language, perception, grounding, planning, and control. Organize code into packages: language (ASR adapters, LLM adapters, dialog manager), perception (object detectors, segmentation, CLIP encoders), grounding (referent matchers, spatial transformers), action (affordance predictors, motion primitive library), and runtime (task manager, execution server). Use ROS 2 actions for long‑running tasks and message types with confidence fields and provenance metadata.

Language stack: prefer modular adapters that convert raw ASR/text to structured intents. Use LLMs or sequence models for decomposition; deploy small on‑device models or quantized adapters (LoRA/INT8) when GPUs are constrained. Ensure the language pipeline exposes uncertainty and alternative parses to the task manager for disambiguation.

Perception and grounding: integrate CLIP‑based matching with geometric verification (depth+ICP) to map image referents to 3D poses. Build an affordance server that accepts an object pose and returns grasp patches or action affordances (grasp, push, pull, open). For learned components, version models and store training metadata (seeds, dataset versions) and evaluation artifacts in an experiment registry (Weights & Biases, MLflow).

Action execution: implement a task executor that sequences pre‑grasp, approach, grasp, and verify stages. Use MoveIt 2 for constrained motions and ros2_control for low‑level controllers; implement impedance control for contact robustness. Incorporate safety checks: reachability and collision checks, torque and F/T monitoring, and a clarification loop if referent confidence is below threshold.

Example — Ask‑and‑Pick implementation on a mobile manipulator: Build ROS 2 packages: vla_asr, vla_llm_adapter, vla_perception, vla_grounding, vla_affordance_server, and vla_executor. vla_asr publishes /asr/text; vla_llm_adapter converts text to intent with alternatives; vla_perception publishes /perception/objects; vla_grounding matches language to object poses using CLIP + ICP; vla_affordance_server returns candidate grasp poses which vla_executor sends to MoveIt 2 and ros2_control. If grounding confidence < threshold, vla_executor invokes a clarification action to the user via TTS.

Acceptance checks:
- [ ] Files written under book/docs/chapters/chapter-10/ with five tab files and index
- [ ] Each tab contains 350–450 words and a labeled Example section
- [ ] PHR created recording this generation

Risks and follow-ups:
- ASR errors and ambiguous language require robust clarification strategies.
- Latency from language and perception stacks can violate real‑time budgets; consider model compression and partitioning.
- Sim‑to‑real grounding remains challenging; maintain paired datasets and HIL benches for validation.