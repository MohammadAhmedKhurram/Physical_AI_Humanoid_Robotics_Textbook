# data model: Chapter and related entities

Entities:

- Chapter
  - id: string (slug)
  - title: string
  - order: integer
  - metadata: object (word_counts per tab, learning objectives)
  - tabs: array of Tab

- Tab
  - role: enum (Concept, System, Simulation, Implementation, Integration)
  - learning_objective: string
  - content: markdown string (350–450 words)
  - examples: array of Example

- Example
  - title: string
  - description: markdown (references to tools: ROS 2, Gazebo, Isaac)
  - evidence: optional code snippets or commands

- ValidationReport
  - chapter_id: string
  - status: PASS|FAIL
  - per_rule: list of {rule_id, status, message, line_numbers?}
  - generated_at: ISO timestamp

Validation rules derived from spec:
- Exactly 5 tabs, each with one of the required roles
- Each tab word count between 350 and 450 words
- Each tab includes at least one Example referencing tools
- Metadata includes declared word counts for each tab

Implementation notes:
- ValidationReport should be output as JSON and stored temporarily in CI artifacts for reviewer access.
- A small Python schema and validation harness will enforce these rules.
