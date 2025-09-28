#!/usr/bin/env python3
"""Generate navigation table for README.md based on docs/*.md front matter.

Rules:
- Parse YAML front matter between leading --- lines.
- Expect keys: title, category, order (optional), draft (optional)
- Exclude: problem-template.md and draft:true
- Output a markdown table sorted by category (alpha) then order then title.
- Inject between markers: <!-- NAV_START --> and <!-- NAV_END -->
"""
from __future__ import annotations
import re
import os
import sys
import pathlib
import typing as t
import textwrap
import yaml  # type: ignore

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DOCS_DIR = REPO_ROOT / "docs"
README = REPO_ROOT / "README.md"
NAV_START = "<!-- NAV_START -->"
NAV_END = "<!-- NAV_END -->"

class DocEntry(t.TypedDict, total=False):
    file: str
    title: str
    category: str
    order: int

FRONT_MATTER_RE = re.compile(r"^---\n(.*?)\n---\n(.*)$", re.DOTALL)

def parse_front_matter(path: pathlib.Path) -> tuple[dict, str]:
    text = path.read_text(encoding="utf-8")
    m = FRONT_MATTER_RE.match(text)
    if not m:
        return {}, text
    import yaml  # local import
    meta_raw, body = m.group(1), m.group(2)
    try:
        meta = yaml.safe_load(meta_raw) or {}
    except Exception as e:
        print(f"WARN: Failed to parse front matter in {path.name}: {e}")
        meta = {}
    return meta, body


def collect_docs() -> list[DocEntry]:
    entries: list[DocEntry] = []
    for p in sorted(DOCS_DIR.glob('*.md')):
        if p.name == 'problem-template.md':
            continue
        meta, _ = parse_front_matter(p)
        if meta.get('draft') is True:
            continue
        title = meta.get('title')
        if not title:
            # Fallback: first heading
            for line in p.read_text(encoding='utf-8').splitlines():
                if line.startswith('# '):
                    title = line[2:].strip()
                    break
        if not title:
            title = p.stem
        category = meta.get('category', '未分类')
        order = meta.get('order') or 1000
        entries.append({
            'file': p.name,
            'title': title,
            'category': category,
            'order': order,
        })
    return entries


def build_table(entries: list[DocEntry]) -> str:
    # Group by category
    from collections import defaultdict
    grouped: dict[str, list[DocEntry]] = defaultdict(list)
    for e in entries:
        grouped[e['category']].append(e)
    lines = ["| 分类 | 问题 | 链接 |", "|------|------|------|"]
    for cat in sorted(grouped.keys()):
        for e in sorted(grouped[cat], key=lambda d: (d['order'], d['title'].lower())):
            slug = e['file'][:-3]
            lines.append(f"| {cat} | {e['title']} | [{slug}](docs/{e['file']}) |")
    return "\n".join(lines)


def inject_table(readme_text: str, table_md: str) -> str:
    marker_present = NAV_START in readme_text and NAV_END in readme_text
    if not marker_present:
        # append at end before License or just end
        appended = f"\n{NAV_START}\n{table_md}\n{NAV_END}\n"
        return readme_text.rstrip() + appended
    pattern = re.compile(re.escape(NAV_START) + r".*?" + re.escape(NAV_END), re.DOTALL)
    return pattern.sub(f"{NAV_START}\n{table_md}\n{NAV_END}", readme_text)


def main() -> int:
    entries = collect_docs()
    if not entries:
        print("No docs found.")
        return 1
    table = build_table(entries)
    readme = README.read_text(encoding='utf-8')
    new_readme = inject_table(readme, table)
    if new_readme != readme:
        README.write_text(new_readme + "\n", encoding='utf-8')
        print("README navigation updated.")
    else:
        print("README already up to date.")
    return 0

if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
