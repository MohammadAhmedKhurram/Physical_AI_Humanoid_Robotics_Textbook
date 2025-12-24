<!--
Sync Impact Report
- Version change: placeholder → 1.0.0
- Modified principles: placeholders replaced with concrete constitution text from user input
- Added sections: Chapter Structure Covenant, AI Agent Governance, Validation & Enforcement, Platform & Permanence, Judge-Facing Transparency
- Removed sections: none
- Templates reviewed: .specify/templates/plan-template.md (⚠ pending explicit gate updates), .specify/templates/spec-template.md (✅ reviewed), .specify/templates/tasks-template.md (✅ reviewed), .specify/templates/phr-template.prompt.md (✅ reviewed), .specify/templates/checklist-template.md (✅ reviewed)
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Confirm and fill ratification date
  - Update plan-template.md "Constitution Check" with explicit gates extracted from this constitution
  - CI validation rules: ensure automated validators enforce Article V chapter structure and Article VII validation requirements
-->

# Physical AI & Humanoid Robotics - Specs‑Driven Textbook System Constitution

## Core Principles

### I — Purpose & Authority
This Constitution defines the supreme governing rules for the design, generation, validation, and publication of the Physical AI & Humanoid Robotics textbook system. All participants — human contributors, AI agents, CI systems, and reviewers — are subordinate to this document. If any instruction, prompt, or action conflicts with this Constitution, this Constitution prevails.

### II — Vision & Mandate
2.1 Vision
The system SHALL produce a course‑grade, intentional, and trustworthy textbook on Physical AI and Humanoid Robotics, demonstrating how Specs‑Driven Development governs AI‑authored systems.

2.2 Mandate
The system MUST:
- Teach embodied intelligence and humanoid robotics rigorously
- Feel human‑designed, not auto‑generated
- Make AI behavior inspectable, auditable, and constrained
- Be understandable by judges within 60 seconds

### III — Scope of Authority
3.1 In‑Scope
- Educational content on Physical AI & Humanoid Robotics
- Course‑aligned modules (ROS 2, Simulation, Isaac, VLA)
- Static documentation published via Docusaurus
- AI‑assisted content generation governed by specs

3.2 Out‑of‑Scope
- Hardware assembly manuals
- Marketing or promotional content
- Free‑form or speculative writing
- Agent autonomy beyond defined specs

### IV — Supremacy of Specifications
4.1 Specs Are Law
All behavior of the system SHALL be governed by written specifications. No content MAY be generated, modified, or published without:
- An applicable spec
- Successful validation against that spec

4.2 No Prompt Authority
Prompts have zero authority. If a prompt contradicts a spec, the spec SHALL be followed and the prompt SHALL be rejected.

### V — Chapter Structure Covenant
Every chapter in the system MUST conform to the following immutable structure:
- Exactly five (5) tabs
- Each tab contains 350–450 words
- Each tab includes at least one clearly labeled "Example" section
- Word counts MUST be declared in metadata
- Structure MUST be validated before publication
Deviation from this structure is constitutionally prohibited.

### VI — AI Agent Governance
6.1 Agent Roles
AI agents are specialized, non‑sovereign workers. They MAY:
- Generate content strictly within spec boundaries
- Analyze and validate content against QA rules
- Report failures with diagnostics

They MAY NOT:
- Modify specifications
- Create or remove tabs
- Override validation
- Exercise creative discretion beyond specs

6.2 Spec Mutation Rule
Any action that would change a spec REQUIRES:
- A human‑authored pull request
- Explicit human review and approval

### VII — Validation & Enforcement
7.1 Mandatory Validation
All generated content MUST pass automated validation before being considered valid. Validation SHALL check:
- Structural integrity
- Word count compliance
- Presence of required sections
- Metadata completeness

7.2 CI Enforcement
Continuous Integration SHALL:
- Block publication of invalid content
- Fail builds on structural violations
- Surface clear diagnostic messages
No exceptions.

### VIII — Platform & Permanence
8.1 Publishing Platform
The system SHALL be deployable as a static Docusaurus site. All content SHALL be authored in Markdown.

8.2 Reproducibility
At any point, a third party MUST be able to:
- Inspect the specs
- Regenerate the content
- Reproduce the same structure and guarantees

### IX — Judge‑Facing Transparency
The system MUST explicitly expose:
- Its Specs‑Driven nature
- Its AI governance model
- Its validation process
Judges MUST NOT be required to infer how the system works. Understanding SHALL be immediate and intentional.

### X — Amendments
This Constitution MAY be amended only if:
- A written proposal is submitted
- The impact on agents, validation, and content is analyzed
- A human reviewer explicitly approves the change
Amendments take effect only after approval.

### XI — Final Clause
This system exists to prove a single principle:
When specifications lead, AI becomes reliable.
When specs disappear, quality collapses.
This Constitution ensures the former — permanently.


**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE) | **Last Amended**: 2025-12-22
