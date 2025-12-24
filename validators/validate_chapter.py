#!/usr/bin/env python3
"""
Chapter validator with role-aware per-tab parsing and rule checks.
Usage: python3 validators/validate_chapter.py specs/<chapter>/spec.md --output output/validation/<chapter>.json

Rules checked:
- R_EXACT_TABS: find exactly 5 tab headings that correspond to required roles
- R_REQUIRED_ROLES: each required role appears exactly once
- R_TAB_WORDCOUNT: each tab content has 350-450 words
- R_EXAMPLE_PRESENT: each tab contains a '### Example' section referencing at least one valid tool (ROS 2, Gazebo, Isaac)
"""
import sys
import json
import argparse
from pathlib import Path
import re

RULES = [
    {"id": "R_EXACT_TABS", "desc": "Exactly 5 tab headings corresponding to roles must be present"},
    {"id": "R_REQUIRED_ROLES", "desc": "All required tab roles must be present exactly once"},
    {"id": "R_TAB_WORDCOUNT", "desc": "Each tab must be 350-450 words"},
    {"id": "R_EXAMPLE_PRESENT", "desc": "Each tab must contain at least one Example referencing valid tools"},
]

TAB_ROLES = ["Concept", "System", "Simulation", "Implementation", "Integration"]
TOOL_TOKENS = ["ros 2", "ros2", "gazebo", "isaac", "nvidia isaac", "ros"]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("spec_path", help="Path to spec.md for a chapter")
    p.add_argument("--output", default="output/validation/report.json", help="Output JSON path")
    return p.parse_args()


def extract_content_after_frontmatter(text: str) -> str:
    m = re.match(r"^---\s*\n(.*?)\n---\s*\n", text, re.S)
    if m:
        return text[m.end():]
    return text


def find_role_tabs(text: str):
    """Return ordered list of tabs that match required roles.
    Each entry: {role, heading, body, start_line, end_line}
    """
    body_text = extract_content_after_frontmatter(text)
    # find all '## ' headings with their positions
    headings = list(re.finditer(r"(?m)^##\s+(.+)$", body_text))
    tabs = []
    for i, m in enumerate(headings):
        heading = m.group(1).strip()
        start = m.end()
        end = headings[i+1].start() if i+1 < len(headings) else len(body_text)
        section = body_text[start:end].strip()
        # detect role by checking for exact role word in heading
        matched_role = None
        for role in TAB_ROLES:
            if re.search(r"\b" + re.escape(role) + r"\b", heading, re.IGNORECASE):
                matched_role = role
                break
        if matched_role:
            tabs.append({"role": matched_role, "heading": heading, "body": section})
    return tabs


def words_count(s: str) -> int:
    # count word-like tokens
    return len(re.findall(r"\w+", s))


def find_example_section(tab_body: str):
    # locate '### Example' or '### Examples' and return its text
    m = re.search(r"(?m)^###\s*Example(s)?\s*$", tab_body, re.IGNORECASE)
    if not m:
        return None
    start = m.end()
    # find next '### ' heading or end
    m2 = re.search(r"(?m)^###\s+", tab_body[start:])
    end = start + m2.start() if m2 else len(tab_body)
    return tab_body[start:end].strip()


def example_references_tool(example_text: str) -> bool:
    if not example_text:
        return False
    lower = example_text.lower()
    for t in TOOL_TOKENS:
        if t in lower:
            return True
    return False


def run_validation(text: str, chapter_id: str):
    per_rule = []
    tabs = find_role_tabs(text)

    # R_EXACT_TABS: exactly 5 role tabs found
    per_rule.append({
        "rule_id": "R_EXACT_TABS",
        "status": "PASS" if len(tabs) == 5 else "FAIL",
        "message": f"Found {len(tabs)} role-based tabs (expected 5)",
    })

    # R_REQUIRED_ROLES: check that all roles are present exactly once
    roles_found = [t["role"] for t in tabs]
    missing = [r for r in TAB_ROLES if r not in roles_found]
    duplicates = [r for r in set(roles_found) if roles_found.count(r) > 1]
    per_rule.append({
        "rule_id": "R_REQUIRED_ROLES",
        "status": "PASS" if (not missing and not duplicates) else "FAIL",
        "message": (
            "All roles present" if (not missing and not duplicates) else
            f"Missing roles: {missing}; Duplicates: {duplicates}"
        ),
        "details": {"roles_found": roles_found},
    })

    # R_TAB_WORDCOUNT: each tab 350-450 words
    tab_word_results = []
    for t in tabs:
        wc = words_count(t["body"])
        ok = 350 <= wc <= 450
        tab_word_results.append({"role": t["role"], "words": wc, "ok": ok})
    all_ok = all(r["ok"] for r in tab_word_results) if tab_word_results else False
    per_rule.append({
        "rule_id": "R_TAB_WORDCOUNT",
        "status": "PASS" if all_ok else "FAIL",
        "message": f"Per-tab word counts: {[r['words'] for r in tab_word_results]}",
        "details": tab_word_results,
    })

    # R_EXAMPLE_PRESENT: each tab has Example section referencing a valid tool
    example_results = []
    for t in tabs:
        ex = find_example_section(t["body"])
        has_example = ex is not None
        refs_tool = example_references_tool(ex) if ex else False
        example_results.append({"role": t["role"], "has_example": has_example, "refs_tool": refs_tool})
    all_examples_ok = all(r["has_example"] and r["refs_tool"] for r in example_results) if example_results else False
    per_rule.append({
        "rule_id": "R_EXAMPLE_PRESENT",
        "status": "PASS" if all_examples_ok else "FAIL",
        "message": "Per-tab example presence and tool references",
        "details": example_results,
    })

    status = "PASS" if all(r["status"] == "PASS" for r in per_rule) else "FAIL"
    report = {"chapter_id": chapter_id, "status": status, "per_rule": per_rule}
    return report


def main():
    args = parse_args()
    spec_path = Path(args.spec_path)
    if not spec_path.exists():
        print(f"Spec path not found: {spec_path}")
        sys.exit(2)
    text = spec_path.read_text(encoding="utf-8")
    chapter_id = spec_path.parent.name
    report = run_validation(text, chapter_id)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
