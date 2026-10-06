import json

with open(r'C:\Users\30496\Desktop\records-2026-10-06-16-45-21.json', 'r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Loaded {len(raw)} bytes.")

# Let's test various transformations
# Known substrings we might expect: '[IMU]', 'Pitch', 'Roll', 'Yaw', 'CALIB', 'WAIT'
keywords = [b'IMU', b'Pitch', b'Roll', b'Yaw', b'deg', b'CALIB', b'WAIT']

# Let's check transformations:
# 1. (b & 0x3F) | ((b & 0x80) >> 1)
# 2. b ^ mask for mask in 0..255
# 3. bit rotations
# 4. invert bits
# 5. baud rate mismatch (e.g. 115200 sent, 9600 sampled)

def score(buf):
    text = buf.decode('latin1', errors='ignore')
    return sum(text.count(kw.decode('latin1')) for kw in keywords)

print("Formula 1:")
f1 = bytes([(b & 0x3F) | ((b & 0x80) >> 1) for b in raw])
print(f"Score: {score(f1)}")
for kw in keywords:
    if kw in f1:
        print(f"Found {kw} in f1!")

# Try XOR
for mask in range(256):
    fx = bytes([b ^ mask for b in raw])
    sc = score(fx)
    if sc > 0:
        print(f"XOR {hex(mask)}: score {sc}")

# Try bit shifts / inversions
for shift in range(1, 8):
    # Left shift / right shift stream of bits
    pass

# Print some decoded hex chunks with formula 1
print("\nFirst 200 bytes with formula 1:")
print(f1[:200].decode('latin1', errors='replace'))
