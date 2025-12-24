# tasks: Complete specs set — Physical AI & Humanoid Robotics

Feature: `001-add-specs-set`
Generated: 2025-12-22

Summary:
- Purpose: Produce an immediately-executable tasks.md that lets an LLM or developer implement the canonical book skeleton, validation pipeline, templates, and reviewer checklists.
- Organization: Phase 1 (Setup), Phase 2 (Foundational), Phase 3+ (User Stories in priority order), Final Phase (Polish & Cross-cutting)

Phase 1 — Setup (project initialization)

- [ ] T001 Create canonical book skeleton (directories and root files) at `book/` and root files `docusaurus.config.js`, `package.json`, `README.md`, `sidebars.js`
- [P] [ ] T002 Create validation contract JSON schema at `specs/001-add-specs-set/contracts/chapter-schema.json`
- [P] [ ] T003 Create initial validator script at `validators/validate_chapter.py` (outputs JSON report to `output/validation/`)
- [P] [ ] T004 Create quickstart documentation at `specs/001-add-specs-set/quickstart.md` describing how to run generators and validators
- [P] [X] T005 Add CI workflow stub to run validators at `.github/workflows/validate-specs.yml`

Phase 2 — Foundational (blocking prerequisites)

- [X] T006 Review & finalize data model at `specs/001-add-specs-set/data-model.md` (ensure entities: Chapter, Tab, Example, ValidationReport are complete)
- [X] T007 Create reviewer checklist file at `specs/001-add-specs-set/checklists/requirements.md` (maps to SPEC-QA-001 rules)
- [X] T008 Create template generator script at `scripts/generate_chapter_template.py` (outputs `specs/<chapter>/template.md`)
- [X] T009 Add documentation entry linking quickstart and validator in `README.md`

Phase 3 — User Story Phases (priority-ordered)

US1 — Author: Create canonical book skeleton (Priority: P1)

- [X] T010 [US1] Create an init script to instantiate the canonical skeleton at `scripts/init_book_skeleton.py` (idempotent; creates `book/` structure and root files)
- [P] [X] T011 [US1] Add an integration test that verifies filesystem layout at `tests/integration/test_skeleton.py` (asserts paths and required root files)
- [X] T012 [US1] Commit canonical chapter index file at `specs/001-add-specs-set/chapter_index.md` containing the SPEC-CHAPTER-MAP-001 list
- [X] T013 [US1] Implement pre-merge validation rule for new chapters in `validators/check_chapter_index.py` (reject non-canonical titles)

US2 — Reviewer: Validate chapter and tab compliance (Priority: P2)

- [X] T014 [US2] Implement rule checks in `validators/validate_chapter.py`: exactly 5 tabs, required roles, per-tab word counts (350–450), and at least one Example referencing valid tools
- [X] T015 [US2] Add unit tests for validator rules at `tests/unit/test_validate_chapter.py` (cover PASS and FAIL cases from spec acceptance scenarios)
- [X] T016 [US2] Integrate validator into CI workflow `.github/workflows/validate-specs.yml` so validation runs on changed `specs/` files and uploads `output/validation/*.json` as artifacts

US3 — Integrator: Generate chapter templates and QA checklist (Priority: P3)

- [X] T017 [US3] Implement the template generator at `scripts/generate_template.py` that writes `specs/<chapter>/spec.md` and `specs/<chapter>/checklists/requirements.md`
- [X] T018 [US3] Add a sample template for Chapter 1 at `specs/chapter-01/template.md` filled with metadata fields required by SPEC-TAB-CONTENT-001
- [X] T019 [US3] Ensure quickstart links to template generator and shows example command in `specs/001-add-specs-set/quickstart.md`

Final Phase — Polish & Cross-cutting Concerns

- [X] T020 Create reviewer rubric file at `specs/001-add-specs-set/checklists/reviewer_rubric.md` (maps to SPEC-REVIEWER-CHECKLIST-001)
- [X] T021 Ensure validation output is machine-readable and stored under `output/validation/<chapter>-report.json` and document artifact location in `.github/workflows/validate-specs.yml`

Dependencies (story completion order):
- Phase 1 tasks (T001—T005) must be completed before Phase 2 tasks that rely on files produced by setup.
- Phase 2 tasks (T006—T009) must complete before US1 (T010—T013) and US2 (T014—T016) implementation tasks that rely on contracts and templates.
- US1 (T010—T013) is the recommended MVP to unblock authors; US2 and US3 can proceed in parallel once foundational artifacts exist.

Parallel execution examples:
- Run T002, T003, T004, T005 in parallel (they touch independent files and set up validation infrastructure).
- While T003 (validator script) is being implemented, a developer can implement T017 (template generator) in parallel because templates and validator rules are independent until integration.

Validation & Acceptance checks (per user story):
- US1 acceptance test: `tests/integration/test_skeleton.py` passes and `validators/validate_chapter.py` returns PASS for the canonical chapter examples.
- US2 acceptance test: `tests/unit/test_validate_chapter.py` includes at least one PASS case (valid chapter) and one FAIL case (missing tab) with diagnostic messages.
- US3 acceptance test: Running `scripts/generate_template.py chapter-01` produces `specs/chapter-01/spec.md` and `specs/chapter-01/checklists/requirements.md`, and the validator marks the sample as PASS.

Implementation strategy (MVP-first):
1. MVP scope: Complete US1 (T010—T013) so authors can start writing chapters. This includes T001 (skeleton) and the minimal validator stub T003 that checks existence of files.
2. Next: Implement US2 (validation rules) and wire CI (T014—T016).
3. Then: Implement US3 (template generation) and reviewer rubric (T017—T019, T020).

Notes & non-goals:
- This task set does NOT include creating commits or opening PRs; follow the user's instruction for git operations.
- Do NOT auto-modify existing specs; all spec changes must be via human-authored PRs per Constitution rules.

Files to be created/modified by these tasks (explicit):
- `specs/001-add-specs-set/tasks.md` (this file)
- `specs/001-add-specs-set/contracts/chapter-schema.json`
- `validators/validate_chapter.py`
- `scripts/generate_template.py`
- `scripts/init_book_skeleton.py`
- `specs/001-add-specs-set/checklists/requirements.md`
- `.github/workflows/validate-specs.yml`

MVP suggestion:
- Implement T001, T003 (minimal validator), T010, T011, T012 for the smallest usable increment that passes the repository validation tool and allows authors to begin work.

Parallel opportunities summary:
- T002-T005: parallel
- T011 (integration test) can be written while T010 is implemented (test-first)
- Template development (T017) can proceed concurrently with validator unit tests (T015)

Total tasks: 21
Tasks per user story:
- US1: 4 tasks (T010–T013)
- US2: 3 tasks (T014–T016)
- US3: 3 tasks (T017–T019)


---

Acceptance checklist for this tasks.md (self-check):
- [ ] All tasks follow the required checklist format (checkbox, TaskID, optional [P], story label when required, and file path)
- [ ] Tasks are organized by Phase and by User Story in priority order
- [ ] Dependencies and parallel execution examples included
- [ ] Validation/acceptance criteria present for each user story



