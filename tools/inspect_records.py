import json
from input_path import require_input_path

path = require_input_path("List record timestamps and lengths from a serial-record JSON file.")
with path.open('r', encoding='utf-8') as f:
    records = json.load(f)

print("Total records:", len(records))
for i, r in enumerate(records):
    print(f"Record {i:02d}: time={r.get('time')}, len={r.get('byteLength')}")
