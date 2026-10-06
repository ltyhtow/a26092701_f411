import json
import struct

with open(r'C:\Users\30496\Desktop\records-2026-10-06-16-45-21.json', 'r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Total raw bytes: {len(raw)}")

# Search for LwPKT frames:
# LwPKT packet structure:
# START byte: 0xAA
# CMD (varint encoded, for 0x84, 0x85, 0x86 it might be 0x84, 0x01 or similar)
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
