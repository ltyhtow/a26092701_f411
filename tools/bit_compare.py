# Plaintext bytes
plain = b"[IMU] Pi"
# Cipher bytes
cipher = bytes([0x84, 0xC0, 0x67, 0x0C, 0x2E, 0xC0, 0xC6, 0x50])

print("Plaintext:")
for ch, b in zip(plain, plain):
    print(f"'{chr(ch)}' (0x{b:02X}) -> {b:08b} (LSB first: {bin(b)[2:].zfill(8)[::-1]})")

print("\nCipher:")
for b in cipher:
    print(f"0x{b:02X} -> {b:08b} (LSB first: {bin(b)[2:].zfill(8)[::-1]})")
