import json
from input_path import require_input_path

path = require_input_path("Compare two candidate 41-byte windows at offsets 80 and 121 in serial-record JSON.")
with path.open('r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

# Compare candidate windows only; repetition does not establish valid framing.
chunk1 = raw[80:121]
chunk2 = raw[121:162]

print("Chunk length:", len(chunk1))
print("Chunk1 hex:", chunk1.hex(' '))

# Let's compare byte by byte between chunk1 and chunk2
if len(chunk1) != 41 or len(chunk2) != 41:
    print("Input is shorter than 162 bytes; comparing only the available overlap.")
diffs = [i for i in range(min(len(chunk1), len(chunk2))) if chunk1[i] != chunk2[i]]
print("Differences at indices:", diffs)
for d in diffs:
    print(f"Index {d}: chunk1=0x{chunk1[d]:02X}, chunk2=0x{chunk2[d]:02X}")
