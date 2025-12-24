#!/usr/bin/env python3
"""
Generate a canonical chapter template.
Usage:
  python3 scripts/generate_chapter_template.py chapter-01 --title "Foundations of Physical AI" --order 1

Writes: specs/<chapter>/template.md
"""
from pathlib import Path
import argparse

TABS = [
    ("Concept", "Theory & definitions"),
    ("System", "Architecture & components"),
    ("Simulation", "Digital twin & environment modeling"),
    ("Implementation", "ROS 2 / Isaac / pipelines"),
    ("Integration", "How this connects to the humanoid & capstone"),
]

FRONTMATTER = """---
id: {id}
title: "{title}"
order: {order}
metadata:
  word_counts:
    Concept: 400
    System: 400
    Simulation: 400
    Implementation: 400
    Integration: 400
---

"""

TEMPLATE_TAB = """
## {role} ({desc})

### Learning objective

- _Add 1–2 learning objectives for this tab._

### Content

_Write 350–450 words here, providing physical grounding and system-level explanation._

### Example

_Provide at least one Example referencing tools (e.g., ROS 2, Gazebo, Isaac)._

"""


def build_template(chapter_id: str, title: str, order: int) -> str:
    body = FRONTMATTER.format(id=chapter_id, title=title, order=order)
    for role, desc in TABS:
        body += TEMPLATE_TAB.format(role=role, desc=desc)
    return body


def main():
    p = argparse.ArgumentParser(description="Generate a canonical chapter template")
    p.add_argument("chapter", help="Chapter slug (e.g., chapter-01)")
    p.add_argument("--title", default=None, help="Printable title for the chapter")
    p.add_argument("--order", type=int, default=1, help="Chapter order integer")
    args = p.parse_args()

    chapter = args.chapter
    title = args.title or args.chapter
    order = args.order

    out_dir = Path("specs") / chapter
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "template.md"
    out_path.write_text(build_template(chapter, title, order), encoding="utf-8")
    print(f"Wrote template: {out_path}")


if __name__ == "__main__":
    main()
