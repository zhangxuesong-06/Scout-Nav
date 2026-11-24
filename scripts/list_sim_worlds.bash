#!/usr/bin/env bash
set -e
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 - "$SCOUT_ROOT/src/scout_sim/ASSET_MANIFEST.json" <<'PY'
import json, sys
for key, entry in json.load(open(sys.argv[1]))['worlds'].items():
    print(key)
    for note in entry['notes']:
        if 'actors replaced' in note or 'wrapper' in note:
            print('    ' + note)
PY
printf 'mapping_course\n    RGB-D loop-closure and grape handoff test course\n'
