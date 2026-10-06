# Check baud rate resampling simulation
# If sender transmits at B_tx, and receiver samples at B_rx:
def resample_uart(tx_bytes, baud_tx, baud_rx):
    # generate bit stream with 1 start bit, 8 data bits (LSB), 1 stop bit
    bits = []
    for b in tx_bytes:
        bits.append(0) # start bit
        for bit_idx in range(8):
            bits.append((b >> bit_idx) & 1)
        bits.append(1) # stop bit (idle high)
        bits.append(1) # inter-byte idle

    # Time per tx bit
    t_tx = 1.0 / baud_tx
    t_rx = 1.0 / baud_rx

    # Total duration
    total_time = len(bits) * t_tx

    # Receiver samples:
    # A standard UART receiver detects falling edge (idle 1 -> start 0),
    # waits 0.5 bit time, then samples every 1.0 bit time for 8 bits, then stop bit.
    rx_bytes = []
    t = 0.0

    def get_line_level(time_sec):
        idx = int(time_sec / t_tx)
        if idx < len(bits):
            return bits[idx]
        return 1

    while t < total_time - 10 * t_rx:
        # wait for falling edge
        while t < total_time and get_line_level(t) == 1:
            t += t_rx * 0.1
        if t >= total_time:
            break
        # Detected start bit at t
        t_start = t
        # Sample middle of data bits: start + 1.5, 2.5, ..., 8.5
        b_val = 0
        for i in range(8):
            sample_t = t_start + (1.5 + i) * t_rx
            lvl = get_line_level(sample_t)
            b_val |= (lvl << i)
        rx_bytes.append(b_val)
        # Advance t past stop bit
        t = t_start + 9.5 * t_rx
    return rx_bytes

target_line = b"[IMU] Pitch:   0.00 | Roll:   0.00 | Yaw:   0.00 (deg)\r\n"

# Test common bauds
bauds = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
print("Testing baud rate combinations...")
for b_tx in [115200]:
    for b_rx in bauds:
        res = resample_uart(target_line, b_tx, b_rx)
        # Compare with cipher snippet: 84 c0 67 0c 2e c0 c6 50
        res_hex = ' '.join(f'{x:02x}' for x in res[:10])
        print(f"TX={b_tx}, RX={b_rx} ({len(res)} bytes): {res_hex}")
