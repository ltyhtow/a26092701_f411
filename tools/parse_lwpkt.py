import json
from input_path import require_input_path

path = require_input_path("Inspect candidate LwPKT marker bytes in serial-record JSON; no CRC validation.")
with path.open('r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Total raw bytes: {len(raw)}")

# Candidate markers only; this script does not reconstruct or validate frames.
# Current firmware packet structure (see lwpkt_opts.h):
# START byte: 0xAA
# CMD (one byte; extended commands are disabled)
# LEN (varint)
# DATA
# CRC8
# STOP byte: 0x55

# Let's search for 0xAA in raw
aa_indices = [i for i, b in enumerate(raw) if b == 0xAA]
print(f"Count of 0xAA (START byte): {len(aa_indices)}")

# If 0xAA is not directly found, maybe bluetooth inverted or shifted it?
# Let's search for 0x84, 0x85, 0x86 positions
pos_84 = [i for i, b in enumerate(raw) if b == 0x84]
pos_85 = [i for i, b in enumerate(raw) if b == 0x85]
pos_86 = [i for i, b in enumerate(raw) if b == 0x86]

print(f"0x84 count: {len(pos_84)}, 0x85 count: {len(pos_85)}, 0x86 count: {len(pos_86)}")

if pos_86:
    for idx in pos_86[:5]:
        chunk = raw[idx:idx+60]
        print(f"0x86 at {idx}: {chunk.hex(' ')}")

if pos_84:
    for idx in pos_84[:5]:
        chunk = raw[idx:idx+35]
        print(f"0x84 at {idx}: {chunk.hex(' ')}")
