import os
from pathlib import Path


def test_book_skeleton_exists():
    root = Path(__file__).resolve().parents[2]
    assert (root / 'book').exists()
    assert (root / 'docusaurus.config.js').exists()
    assert (root / 'package.json').exists()
    assert (root / 'sidebars.js').exists()


def test_chapter_template_created():
    assert (Path('specs/chapter-01/template.md')).exists()
