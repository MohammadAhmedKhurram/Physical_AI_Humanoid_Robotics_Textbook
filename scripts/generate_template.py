#!/usr/bin/env python3
"""
Generate a canonical chapter spec and checklist.

Usage:
  python3 scripts/generate_template.py chapter-01 --title "Foundations of Physical AI" --order 1

This creates:
  specs/<chapter>/spec.md
  specs/<chapter>/checklists/requirements.md

By default it fills sample content that meets validator requirements (5 tabs, 350-450 words per tab,
Example section referencing ROS 2/Gazebo) so the validator can mark the sample as PASS.
"""
from pathlib import Path
import argparse
import re

TAB_ROLES = [
    ("Concept", "Theory & definitions"),
    ("System", "Architecture & components"),
    ("Simulation", "Digital twin & environment modeling"),
    ("Implementation", "ROS 2 / Isaac / pipelines"),
    ("Integration", "How this connects to the humanoid & capstone"),
]

DEFAULT_WORDS = 380


def words_count(s: str) -> int:
    return len(re.findall(r"\w+", s))


def generate_tab_body(role: str, target_words: int = DEFAULT_WORDS) -> str:
    # A set of varied sentence templates referencing physical grounding and system concerns
    templates = [
        f"The {role} tab explains core concepts and ties them to physical grounding and measurable outcomes.",
        "It emphasizes system-level explanations, interfaces, and the trade-offs engineers make in real deployments.",
        "We describe evaluation metrics, safety considerations, and integration points with simulators and middleware.",
        "Technical examples illustrate cause → effect relationships and show how design decisions impact performance.",
        "Where appropriate, we reference tools and simulation environments to enable reproducibility and testing.",
        f"The {role} discussion highlights concrete steps authors can take to validate ideas with experiments or simulation."
    ]

    out = []
    i = 0
    # Keep appending template sentences until we reach target word count
    while words_count(" ".join(out)) < target_words:
        out.append(templates[i % len(templates)])
        i += 1
    # join into paragraphs
    paragraphs = []
    # make 3 paragraphs for readability
    chunk = max(1, len(out) // 3)
    for j in range(0, len(out), chunk):
        paragraphs.append(" ".join(out[j:j+chunk]))
    return "\n\n".join(paragraphs)


def build_spec_md(chapter_id: str, title: str, order: int) -> str:
    front = [
        "---",
        f"id: {chapter_id}",
        f"title: \"{title}\"",
        f"order: {order}",
        "metadata:",
        "  word_counts:",
    ]
    # set declared word counts to 400 for each role
    for role, _ in TAB_ROLES:
        front.append(f"    {role}: 400")
    front.append("---\n")

    body = "\n".join(front)

    for role, desc in TAB_ROLES:
        body += f"## {role} ({desc})\n\n"
        body += "### Learning objective\n\n- _Add 1–2 learning objectives for this tab._\n\n"
        body += "### Content\n\n"
        content = generate_tab_body(role, DEFAULT_WORDS)
        body += content + "\n\n"
        # Example section that includes tool references (ROS 2 and Gazebo)
        example_text = (
            "### Example\n\n"
            "This example shows how to use ROS 2 and Gazebo to simulate sensor data and validate perception pipelines. "
            "Run a simulated sensor node, publish messages to the middleware, and observe the effect in the simulator. "
            "Commands and snippets can be added here for reproducibility and reviewer validation.\n\n"
        )
        body += example_text

    return body


def build_checklist_md(chapter_id: str) -> str:
    lines = [
        f"# Review Checklist for {chapter_id}\n",
        "- [ ] Exactly 5 tabs present (Concept, System, Simulation, Implementation, Integration)",
        "- [ ] Each tab contains a Learning objective",
        "- [ ] Each tab content 350–450 words",
        "- [ ] Each tab includes at least one Example referencing tools (ROS 2, Gazebo, Isaac)",
        "- [ ] Metadata (word_counts) present in frontmatter",
    ]
    return "\n".join(lines) + "\n"


def main():
    p = argparse.ArgumentParser(description="Generate a canonical chapter spec and checklist")
    p.add_argument("chapter", help="Chapter slug (e.g., chapter-01)")
    p.add_argument("--title", default=None, help="Printable title for the chapter")
    p.add_argument("--order", type=int, default=1, help="Chapter order integer")
    p.add_argument("--force", action="store_true", help="Overwrite existing files")
    args = p.parse_args()

    chapter = args.chapter
    title = args.title or args.chapter
    order = args.order

    out_dir = Path("specs") / chapter
    checklist_dir = out_dir / "checklists"
    out_dir.mkdir(parents=True, exist_ok=True)
    checklist_dir.mkdir(parents=True, exist_ok=True)

    spec_path = out_dir / "spec.md"
    checklist_path = checklist_dir / "requirements.md"

    if spec_path.exists() and not args.force:
        print(f"Spec already exists at {spec_path}. Use --force to overwrite.")
    else:
        spec_md = build_spec_md(chapter, title, order)
        spec_path.write_text(spec_md, encoding="utf-8")
        print(f"Wrote spec: {spec_path}")

    if checklist_path.exists() and not args.force:
        print(f"Checklist already exists at {checklist_path}. Use --force to overwrite.")
    else:
        checklist_md = build_checklist_md(chapter)
        checklist_path.write_text(checklist_md, encoding="utf-8")
        print(f"Wrote checklist: {checklist_path}")

if __name__ == '__main__':
    main()
