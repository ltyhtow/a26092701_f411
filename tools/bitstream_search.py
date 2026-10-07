import json
from input_path import require_input_path

path = require_input_path("Search serial-record JSON for shifted byte and bitstream marker patterns.")
with path.open('r', encoding='utf-8') as f:
    records = json.load(f)

raw = bytearray()
for r in records:
    if 'data' in r:
        parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
        raw.extend(parts)

print(f"Loaded {len(raw)} bytes.")

# Construct bitstream LSB first (standard UART transmission order)
bitstream_lsb = ""
bitstream_msb = ""
for b in raw:
    bitstream_lsb += bin(b)[2:].zfill(8)[::-1]
    bitstream_msb += bin(b)[2:].zfill(8)

target_aa_msb = bin(0xAA)[2:].zfill(8) # '10101010'
target_55_msb = bin(0x55)[2:].zfill(8) # '01010101'

print(f"Occurrences of 0xAA in bitstream_msb: {bitstream_msb.count(target_aa_msb)}")
print(f"Occurrences of 0xAA in bitstream_lsb: {bitstream_lsb.count(target_aa_msb)}")
print(f"Occurrences of 0x55 in bitstream_msb: {bitstream_msb.count(target_55_msb)}")
print(f"Occurrences of 0x55 in bitstream_lsb: {bitstream_lsb.count(target_55_msb)}")

# Let's check bit shifts on bytes (0 to 7 bits)
for shift in range(8):
    shifted_bytes = []
    for i in range(len(raw) - 1):
        # combining two adjacent bytes
        val = ((raw[i] << 8) | raw[i+1]) >> shift
        shifted_bytes.append(val & 0xFF)
    c_aa = shifted_bytes.count(0xAA)
    c_55 = shifted_bytes.count(0x55)
    if c_aa > 0 or c_55 > 0:
        print(f"Shift {shift}: 0xAA={c_aa}, 0x55={c_55}")
