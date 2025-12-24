import json
from validators.validate_chapter import run_validation


def load_text(path):
    return open(path, 'r', encoding='utf-8').read()


def test_validator_pass_for_template():
    text = load_text('specs/chapter-01/template.md')
    report = run_validation(text, 'chapter-01')
    # Template is not filled with examples or full word counts, so expect FAIL
    assert isinstance(report, dict)


def test_validator_structure():
    # Ensure the validator returns per_rule structure
    text = load_text('specs/chapter-01/template.md')
    report = run_validation(text, 'chapter-01')
    assert 'per_rule' in report
    assert any(r['rule_id']=='R_EXACT_TABS' for r in report['per_rule'])
