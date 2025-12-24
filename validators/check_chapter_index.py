#!/usr/bin/env python3
"""
Pre-merge check: ensure new chapter titles are in the canonical chapter index
Usage: python3 validators/check_chapter_index.py specs/<chapter>/spec.md --index specs/001-add-specs-set/chapter_index.md
"""
import sys
from pathlib import Path
import argparse


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("spec_path")
    p.add_argument("--index", default="specs/001-add-specs-set/chapter_index.md")
    return p.parse_args()


def get_title_from_spec(path: Path):
    text = path.read_text(encoding='utf-8')
    # naive: first YAML title or first '# ' header
    import re
    m = re.search(r"^title:\s*\"?(.+?)\"?\s*$", text, re.MULTILINE)
    if m:
        return m.group(1).strip()
    m2 = re.search(r"^#\s+(.+)$", text, re.MULTILINE)
    if m2:
        return m2.group(1).strip()
    return None


def load_index(path: Path):
    lines = [l.strip() for l in path.read_text(encoding='utf-8').splitlines() if l.strip()]
    # drop header
    if lines and lines[0].startswith('#'):
        lines = lines[1:]
    # remove numbering
    titles = []
    for l in lines:
        titles.append(l.lstrip('0123456789. ').strip())
    return titles


def main():
    args = parse_args()
    spec = Path(args.spec_path)
    if not spec.exists():
        print(f"Spec not found: {spec}")
        sys.exit(2)
    title = get_title_from_spec(spec)
    if not title:
        print("Could not determine title from spec")
        sys.exit(2)
    index = Path(args.index)
    if not index.exists():
        print(f"Index not found: {index}")
        sys.exit(2)
    titles = load_index(index)
    if title in titles:
        print("OK: Title is canonical")
        sys.exit(0)
    else:
        print("FAIL: Title not in canonical chapter index")
        print("Found title:", title)
        print("Index titles:")
        for t in titles:
            print(" -", t)
        sys.exit(1)

if __name__ == '__main__':
    main()
