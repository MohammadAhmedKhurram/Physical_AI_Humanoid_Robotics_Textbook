---
title: "Chapter 10 — Vision-Language-Action (VLA) Systems: System"
tab: System
word_count: 370
---

A VLA system integrates perception, language processing, planning, and control into a modular architecture with clear interfaces and failure modes. Core subsystems are: language ingestion (ASR/TTS or text I/O), multimodal perception (object detection, instance segmentation, semantic mapping), grounding & reasoning (referent resolution, instruction decomposition), action generation (affordance prediction, motion primitive library), and execution/control (motion planners, compliance controllers).

Communication and contracts use ROS 2 topics/services/actions. Key topics include /asr/text (std_msgs/String), /perception/objects (custom detected object messages with pose and semantic labels), /affordances (affordance maps or grasp candidates), and an /execute_intent action server that sequences planning and control steps. Represent messages with explicit confidence fields so upstream components can decide to proceed, replan, or ask for clarification.

Language modules use pre‑trained LLMs or seq2seq models to parse instructions into structured intents. Use small adapter models or prompt‑tuned LLMs for on‑device inference (e.g., LLaMA variants with LoRA adapters) to reduce latency. Grounding leverages vision‑language encoders (CLIP or unified VL models) to compute similarity between image patches and language tokens; combine with spatial priors (camera intrinsics and TF tree) to map image coordinates to 3D poses consumable by motion planners.

Safety and verification: include a supervisory verifier that checks reachability, collision risk and torque margins before execution. Implement a negotiation channel for ambiguous instructions: if referent confidence < threshold, request clarification through the language channel. For real‑time constraints, partition compute: perception and language on GPU‑accelerated nodes, planning on a higher‑power CPU, and low‑latency controller loops on a real‑time core.

Example — Tabletop fetch task: operator issues “bring me the blue screwdriver on the second shelf.” The ASR publishes text; a LLM adapter decomposes intent into locate → grasp → transport → present. Perception uses an RGB‑D sensor on a pan‑tilt unit to search shelves; CLIP matches language tokens to detected bounding boxes; MoveIt 2 plans a constrained trajectory to retrieve the screwdriver; a compliance controller executes insertion into a handover pose. If the referent ambiguity persists, the system asks “Do you mean the long or short blue screwdriver?” and waits for clarification before proceeding.

Design VLA systems for observability: log utterances, candidate rankings, affordance maps, and execution traces for debugging and improvement.