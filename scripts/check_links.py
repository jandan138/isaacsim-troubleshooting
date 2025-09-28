#!/usr/bin/env python3
"""Basic link existence check for README.md -> docs/*.md
Exit code 1 if any referenced doc file is missing.
"""
from __future__ import annotations
import re, pathlib, sys
root = pathlib.Path(__file__).resolve().parent.parent
readme_path = root / 'README.md'
text = readme_path.read_text(encoding='utf-8')
links = re.findall(r'\]\((docs/[a-zA-Z0-9_.\-/]+\.md)\)', text)
missing: list[str] = []
for rel in links:
    if not (root / rel).exists():
        missing.append(rel)
if missing:
    print('Missing linked docs:', missing)
    sys.exit(1)
print(f'Link check passed: {len(links)} links verified.')
