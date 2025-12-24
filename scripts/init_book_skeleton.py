#!/usr/bin/env python3
"""
Idempotent script to create the canonical book skeleton
Creates: book/, book/blog/, book/docs/, book/src/, book/static/
Creates root files: docusaurus.config.js, package.json, README.md, sidebars.js
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOOK = ROOT / "book"

def write_if_missing(path: Path, content: str = ""):
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def main():
    write_if_missing(BOOK / "blog/.gitkeep", "")
    write_if_missing(BOOK / "docs/.gitkeep", "")
    write_if_missing(BOOK / "src/.gitkeep", "")
    write_if_missing(BOOK / "static/.gitkeep", "")

    # root files
    write_if_missing(ROOT / "docusaurus.config.js", "// docusaurus config placeholder\nmodule.exports = {}\n")
    write_if_missing(ROOT / "package.json", '{"name": "book-site", "private": true}\n')
    write_if_missing(ROOT / "README.md", "# Book site\n")
    write_if_missing(ROOT / "sidebars.js", "module.exports = {}\n")
    print("Book skeleton ensured at: ", BOOK)

if __name__ == '__main__':
    main()
