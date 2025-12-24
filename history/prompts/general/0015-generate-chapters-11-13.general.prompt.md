---
id: 0015
title: Generate Chapters 11–13 - start
stage: general
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-add-specs-set
user: 123
command: /sp.generate chapter-11,chapter-12,chapter-13 --tabs concept,system,simulation,implementation,integration --specs-kit-plus
labels: ["chapter","generation","conversational","sim-to-real","capstone"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - book/docs/chapters/chapter-11/index.md
 - book/docs/chapters/chapter-12/index.md
 - book/docs/chapters/chapter-13/index.md
tests:
 - none
---

## Prompt

Generate Chapters 11, 12, and 13 sequentially. Each chapter must follow Specs‑Kit‑Plus: 5 tabs (Concept, System, Simulation, Implementation, Integration), 350–450 words per tab, ≥1 physically grounded example per tab, reference ROS 2, Gazebo, Unity, NVIDIA Isaac, and real robot toolchains. Write to book/docs/chapters/chapter-11/index.md, chapter-12/index.md, chapter-13/index.md. Preserve sequential order and validate each chapter before proceeding.

## Response snapshot

Started generation of chapters 11–13.
