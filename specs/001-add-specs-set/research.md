# research: Resolve technical unknowns for Complete specs set

Decision: Use Docusaurus for site output and Python-based validators that produce JSON machine-readable validation reports.

Rationale:
- Docusaurus is already referenced in the spec and is suitable for static Markdown sites.
- Python offers rich text-processing libraries (markdown, frontmatter parsing) and existing test tooling (pytest) which fit CI pipelines.
- JSON/YAML validation reports are easy to consume by CI and reviewer tooling.

Alternatives considered:
- Node.js-based validators (remark, unified): good for JS ecosystems; chosen Python for validator expressiveness and existing infra familiarity.
- Using OpenAPI: not applicable to content validation; instead use JSON Schema for document-level validation.

Tasks produced:
- Implement validator script: `validators/validate_chapter.py` (outputs JSON report)
- Create JSON Schema for chapter structure: `specs/001-add-specs-set/contracts/chapter-schema.json`
- Add GitHub Actions CI job to run the validator on changed `specs/` files

Notes: Several items are marked NEEDS CLARIFICATION in the plan (testing framework preference and exact CI recipe). These will be finalized during Phase 1.
