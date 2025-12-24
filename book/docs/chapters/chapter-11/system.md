System — Architecture for Conversational Robotics

Overview
A practical architecture combines perception (microphones, cameras, depth sensors), speech processing (front-end beamforming, acoustic models), natural language understanding, dialogue management, task and motion planning, and actuator control. ROS 2 is used as the integration backbone with modular nodes: audio_capture, beamformer, streaming_asr, nlu, dialogue_manager, semantic_mapper, planner, and safety_monitor. Nodes communicate via DDS topics and services; parameterized components allow deployment across edge devices and a cloud backend for heavy language models when network connectivity and latency budgets permit.

Core components
- Audio pipeline: microphone arrays → beamforming node → voice activity detection (VAD) → streaming ASR (on‑device or cloud hybrid). Use Kaldi/ESPnet or commercial models with a low-latency streaming API.
- Perception & grounding: RGB‑D and object detection nodes publish perceived object poses and tracked person IDs to a semantic_mapper that associates language referring expressions with physical entities.
- NLU & Dialog: NLU converts transcripts to intent and slot structures. Dialog manager maintains conversation state, handles clarification policies, and maps intents to action primitives.
- Planner and Safety: High-level task intents map to navigation or manipulation goals. The planner consults the safety_monitor and local costmaps before issuing velocity commands. Emergency stop is exposed as a high‑priority ROS 2 service.

Data flows and timing constraints
- Streaming constraints require a pipeline capable of sub‑second round trips for simple requests. For complex planning queries it is acceptable to plan asynchronously with the user informed of the delay.
- Use QoS profiles on DDS topics: best-effort for video frames; reliable for command and state messages.

Physically grounded example (assistive home robot)
A home assistant robot runs the audio pipeline on an onboard Jetson device while offloading heavy NLU to a nearby edge server. Spoken command "Bring my cane" triggers ASR -> NLU -> the semantic_mapper to resolve "my cane" to an object pose in the living room. Planner computes a manipulation sequence (navigate -> pick -> deliver) while safety_monitor ensures human proximity is considered. The system illustrates the need for hybrid compute and clear fallbacks when offload fails.