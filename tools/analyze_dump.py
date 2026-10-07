import json
from input_path import require_input_path

path = require_input_path("Inspect byte offsets 100 through 159 of a serial-record JSON file.")
with path.open('r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Total raw bytes: {len(raw)}")
# Print 60 bytes
for i in range(100, min(160, len(raw))):
    b = raw[i]
    ch = chr(b) if 32 <= b <= 126 else '.'
    print(f"{i:03d}: 0x{b:02X}  {b:08b}  {ch}")
