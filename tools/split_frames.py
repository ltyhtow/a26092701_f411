import json

with open(r'C:\Users\30496\Desktop\records-2026-10-06-16-45-21.json', 'r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Total raw bytes: {len(raw)}")

# Let's inspect where 0x84 and 0x86 occur and the distance between them
indices_84 = [i for i, b in enumerate(raw) if b == 0x84]
indices_86 = [i for i, b in enumerate(raw) if b == 0x86]

print("Indices of 0x86 (first 10):", indices_86[:10])
print("Deltas between consecutive 0x86:", [indices_86[i+1] - indices_86[i] for i in range(min(15, len(indices_86)-1))])

# Print 10 slices starting at 0x86
for idx in indices_86[:5]:
    length = 41
    print(f"\nFrame at {idx} (len {length}):")
    print(raw[idx:idx+length].hex(' '))
