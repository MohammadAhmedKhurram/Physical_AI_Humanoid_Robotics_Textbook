# quickstart: generate templates and run validator

1. Generate templates:
   - Run the template generator script to create `specs/<chapter>/spec.md` and `specs/<chapter>/checklists/requirements.md` for each chapter:

     ```sh
     python3 scripts/generate_template.py chapter-01 --title "Foundations of Physical AI" --order 1
     ```

   - The generator can produce sample content that meets validator checks for testing; use `--force` to overwrite existing files.

2. Author a chapter:
   - Fill `specs/<chapter>/spec.md` following the canonical tab structure and metadata.

3. Run validator locally:
   - python3 validators/validate_chapter.py specs/<chapter>/spec.md --output reports/<chapter>.json

4. CI integration:
   - Add a GitHub Actions job that runs the validator on changed `specs/` files and uploads the JSON report as an artifact.

Notes:
- Validators are expected to be deterministic and produce clear per-rule diagnostics suitable for reviewers.
- The template generator and validator scripts are separate tasks (Phase 2/3).
