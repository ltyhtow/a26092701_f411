import json

with open(r'C:\Users\30496\Desktop\records-2026-10-06-16-45-21.json', 'r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

# Let's take one exact 41-byte repeating chunk from raw
# Indices 80 to 121 (length 41):
# 84 c0 67 0c 2e c0 c6 50 bf f0 1e 86 65 0f 1c 88 07 e2 67 3c 63 0e d0 80 94 3c 0e 1e cd c7 56 08 9a 05 3c 43 0e c5 df 0c a4
chunk1 = raw[80:121]
chunk2 = raw[121:162]

print("Chunk length:", len(chunk1))
print("Chunk1 hex:", chunk1.hex(' '))

# Let's compare byte by byte between chunk1 and chunk2
diffs = [i for i in range(41) if chunk1[i] != chunk2[i]]
print("Differences at indices:", diffs)
for d in diffs:
    print(f"Index {d}: chunk1=0x{chunk1[d]:02X}, chunk2=0x{chunk2[d]:02X}")
