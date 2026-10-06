import json

with open(r'C:\Users\30496\Desktop\records-2026-10-06-16-45-21.json', 'r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Total raw bytes: {len(raw)}")
# Print 60 bytes
for i in range(100, 160):
    b = raw[i]
    ch = chr(b) if 32 <= b <= 126 else '.'
    print(f"{i:03d}: 0x{b:02X}  {b:08b}  {ch}")
