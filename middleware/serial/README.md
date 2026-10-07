# Serial protocol boundary

`serial_codec.c` and `serial_protocol_engine.c` are ordinary C11. They include no
STM32, board, FreeRTOS, application queue or control types. LwPKT and LwRB remain
the framing and ring-buffer implementation.

The normal transmit path is:

`native DTO -> serial_codec_encode -> lwpkt_write -> LwRB -> UART DMA / USB CDC`

USB builds select a separate transport backend. USB packet boundaries (64 bytes)
do not change LwPKT framing. A USB session change clears buffered receive bytes
and resets the parser's partial packet; the RTOS adapter rejects dispatch from an
older session. Only the protocol task services the USB stack. Native USB setup,
including the PA11 motor-wire move, is documented in `../../USB_CDC_GUIDE.md`.

The DTOs in `serial_protocol.h` are native in-memory values, not packed wire
structs. `serial_protocol_send` / `serial_protocol_try_send` take the matching
DTO and its native `sizeof`; this size only validates the caller. Encoding
accesses each field explicitly and never copies the native struct to the wire.

## Version 1 wire contract

Every multibyte integer is little-endian; signed integers use two's complement.
Byte 0 is version 1 and byte 1 is sequence. Reserved bytes are emitted as zero
and rejected on receive when nonzero. Existing payload lengths are preserved.

| Command | Payload bytes | Field offsets after version and sequence |
|---|---:|---|
| `01` motion | 16 | reserved 2..3; linear 4; yaw 8; timeout 12; flags 14 |
| `02` PID | 20 | loop 2; reserved 3; Kp 4; Ki 8; Kd 12; integral limit 16 |
| `03` system | 4 | action 2; reserved 3 |
| `04` parameter | 8 | operation 2; key 3; signed Q16.16 value 4 |
| `81` motion telemetry | 32 | status 2; angle 4; gyro 8; left/right speed 12/16; left/right PWM 20/22; battery 24; fault 26; time 28 |
| `82` PID completion / `83` parameter result | 24 | request command 2; result 3; key/loop 4; controller state 5; storage flags 6; value 8; faults 12; revision 16; time 20 |
| `84` IMU sample | 24 | status 2; accel 4/6/8; gyro 10/12/14; error 16; reserved 18..19; time 20 |
| `85` IMU diagnostic | 24 | address 2; identity 3; init result 4; reserved 6..7; HAL status 8; HAL error 12; reserved 16..19; time 20 |
| `86` attitude | 52 | status 2; calibration count 4; reserved 6..7; roll/pitch/yaw 8/12/16; bias 20/24/28; quaternion 32/36/40/44; time 48 |
| `87` encoder test | 56 | flags 2; time 4; period 8; raw L/R 12/16; delta L/R 20/24; GPIO 28; signed 64-bit totals L/R 32/40; dropped 48; errors 52 |

Commands `82` and `83` now carry explicit application results. Sequence echoes
the original request, and `OK` is sent only after completion. A successful queue
submission is not an acknowledgement. `82` uses key as the legacy zero-based
PID loop index; its value and storage flags are zero. `83` reports parameter and
storage state supplied by the parameter worker. Both retain the normal
codec -> LwPKT -> LwRB -> UART DMA send path.

Parameter operations: GET=1, SET=2, SAVE=3, LOAD=4, DEFAULTS=5, STATUS=6.
Result codes: OK=0, BUSY=1, INVALID=2, UNSUPPORTED=3, UNSAFE=4,
STORAGE_ERROR=5, NO_SAVED=6, NOT_READY=7. Unknown operation/key values reach the
worker for an explicit rejection; malformed payload lengths/versions are counted
by the parser and discarded. Firmware updates are not implied by these commands.
Configuration and legacy PID changes apply atomically at a control-cycle boundary
only while DISARMED. ARMED, CALIBRATING and FALLEN reject tuning. A full profile
and a legacy PID in one cycle give the full profile priority and reject PID with
BUSY. Neither kind of update arms the motors. Hosts should issue one request at a
time and treat a missing result as unknown outcome; query STATUS/GET before
deciding whether to repeat a mutating command. Replies can be dropped if TX is
full, since control-task callbacks never block.

Storage/configuration maintenance uses a motor-disable gate. During maintenance,
motion enable and system actions other than DISARM receive `83/BUSY`; legacy PID
receives `82/BUSY`. Gate release clears deferred system/motion queues and keeps
disarm latched. Legacy PID requests that were already queued at release retain
their completion but receive BUSY instead of taking effect afterwards. Legacy
PID gains are limited to magnitude 10000 and integral limit to 0..30000, matching
the persisted profile policy.

IMU diagnostic bytes 18..19 formerly came from ARM
struct padding; they are now explicit zero reserved bytes.

## Ownership and scheduling

### Suspended-wheel diagnostic extension

`0x06` is a four-byte command `[version, sequence, action, reserved=0]`:
START=1, STOP=2, HEARTBEAT=3. Only the WheelTest application handles it; normal
firmware never drives motors because it received this command.

`0x88` is a 32-byte status payload, little endian `BBBBIIhhIHHII`: version,
sequence, phase, reason, timestamp_ms, phase_elapsed_ms, left_pwm, right_pwm,
heartbeat_age_ms, flags, reserved=0, remaining_ms, test_elapsed_ms. Phase and
flag meanings are documented in WHEEL_TEST_GUIDE.md. Encoder `0x87` is unchanged.
Both pass through the same explicit codec, LwPKT and LwRB chain. A received
START is not proof of output; the host requires valid COUNTDOWN status.

The sequence runs once for 21000ms, including 5000ms preparation. Heartbeat
timeout is 750ms; terminal states keep zero output and the motor fault latch
until MCU reset. Existing SYSTEM DISARM and MOTION disable also stop WheelTest.


`serial_engine_init` receives caller-owned initialized RX/TX rings, a typed packet
callback and its context. The engine owns its parser and statistics. Calls must
be serialized by the host. The callback only receives a validated native DTO;
application code translates that DTO into domain commands. Returning false from
the callback increments the queue-overrun counter. Packet pointers are valid
only for the callback duration.

`serial_engine_poll` takes monotonic milliseconds, including unsigned wrap. It
stages at most 128 bytes and processes at most eight parser results per call.
The private staging ring prevents continuous ISR input/noise from keeping the
LwPKT byte parser running indefinitely. Incomplete frames persist between calls.
The caller must poll at least every 100 ms to service parser timeouts.

The FreeRTOS service in `platform/freertos` owns the task, mutex and transport
lifecycle. `serial_service_start` and `serial_service_stop` are supported only
before scheduler startup. Calls during scheduler operation are rejected without
tearing down a live service. Startup failure cleans partial transport state,
task and mutex. The protocol callback must not block; synchronous protocol
submission from that callback recognizes the already-owned mutex.

`try_send` waits zero ticks for the mutex and returns `BUSY` or `TX_FULL` without
queuing a partial frame. Ordinary `send` allows up to 10 ms for the mutex. Both
are task-context APIs. UART callbacks and DMA live in the STM32 platform backend.

## Regression evidence

- `tools/test_serial_codec.c`: legacy and tuning byte-for-byte golden payloads, signed
  extrema, exact lengths, version, reserved/padding zeroes, unsupported commands.
- `tools/test_protocol_engine.c`: real LwPKT/LwRB, fragmented frames, CRC/stop/
  memory errors, timeout resynchronization, dispatch and work budgets, TX capacity.
- `tools/test_serial_protocol_adapter.c`: lifecycle rollback, callback reentry,
  nonblocking submission and 100 Hz time conversion using platform stubs.
- `tools/test_encoder_lwpkt.c` and `tools/test_encoder_monitor.py`: real C encoder
  packet interoperability with the existing Python decoder.
