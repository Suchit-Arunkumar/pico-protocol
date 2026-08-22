# pico-protocol

A binary serial packet protocol for a Raspberry Pi <-> RP2350 (Pico 2) link, built for real-time control of an ROV/AUV thruster system. Fixed 62-byte frames, CRC-16 validated, with a parser on each end that resynchronises from a corrupted or misaligned byte stream instead of stalling. Built standalone, later integrated into Team Tiburon's AUV firmware for SAUVC 2026.

This repository is the protocol layer only — framing, packet structs, CRC, and the receive-side parsers, plus a host-side test harness that runs without hardware. The Pico's control loop and its telemetry transmit path live in the AUV firmware and are not included here.

## Wire format

```
[ STX1 ][ STX2 ][ LEN ][ TYPE ][ ... PAYLOAD (56 bytes) ... ][ CRC_HI ][ CRC_LO ]
  0xAA    0x55    1B     1B              56 bytes                1B       1B
```

- **STX1 / STX2** — fixed 2-byte sync sequence (`0xAA 0x55`)
- **LEN** — payload length in bytes (56 for every packet type below)
- **TYPE** — packet type, see table below
- **PAYLOAD** — type-specific packed struct, little-endian, always 56 bytes
- **CRC** — CRC-16/IBM-3740 over `[LEN, TYPE, PAYLOAD]` — **not** over the sync bytes. Transmitted big-endian (high byte first), unlike the payload. Full parameters below; note it is **not** the KERMIT variant usually meant by "CRC-16-CCITT".

Total packet size: `4 + 56 + 2 = 62 bytes`.

Both sides pin the layout of every wire-visible struct: the C side with `_Static_assert` on `sizeof` and on the `offsetof` of each field, the Python side with equivalent checks at import. The two sets of offsets are checked against each other, so a field reorder that preserves total size — which would otherwise produce a valid-CRC packet decoding to wrong values — fails the build on one end and import on the other.

## Packet types

| Type | Direction | Purpose |
|------|-----------|---------|
| `0x01` TELEMETRY | Pico → Pi | Status snapshot: depth, PID outputs, ESC PWM values, armed/link/saturation flags |
| `0x02` CMD | Pi → Pico | Fused current pose + target pose (6-DOF) + arm state + sequence number |
| `0x03` PID | Pi → Pico | Live PID gain updates (kp/ki or kd/kff), so gains can be retuned without reflashing |

What is actually implemented **in this repo**, per side:

| Type | Pi side (Python) | Pico side (C) |
|------|------------------|---------------|
| `0x01` TELEMETRY | decodes (`read_packet`) | struct defined; transmit path is in the AUV firmware, not here |
| `0x02` CMD | encodes (`build_command_packet`) | parses (`pp_rx_try_parse`) |
| `0x03` PID | no encoder | parses and delivers; applying the gains is in the AUV firmware, not here |

## CRC parameters

Read from the implementation — `crc16()` in [firmware/pico_protocol.h](firmware/pico_protocol.h), mirrored by `crc16_ccitt()` in [python/pico_protocol.py](python/pico_protocol.py) — not from prose.

| Parameter | Value |
|---|---|
| Variant | **CRC-16/IBM-3740** (aka "CRC-16/CCITT-FALSE") |
| Width | 16 bits |
| Polynomial (normal) | `0x1021` |
| Polynomial (reversed) | `0x8408` |
| Initial value | `0xFFFF` |
| Reflect in | No |
| Reflect out | No |
| Final XOR | `0x0000` |
| Check value — CRC of `"123456789"` | **`0x29B1`** |
| Covers | `[LEN, TYPE, PAYLOAD]`, 58 bytes; excludes the sync bytes |
| On-wire byte order | Big-endian: `CRC_HI` then `CRC_LO` |

**This link uses CRC-16/IBM-3740, not the KERMIT variant.** The distinction matters because "CRC-16-CCITT" is used loosely for both, and they do not interoperate:

| | This link — IBM-3740 | "CRC-16-CCITT" — KERMIT |
|---|---|---|
| Init | `0xFFFF` | `0x0000` |
| Reflect in / out | No / No | Yes / Yes |
| Check value of `"123456789"` | `0x29B1` | `0x2189` |

The implementation here computes `0x29B1`, matching IBM-3740's published check value. Anyone writing a third-party client for this link should target IBM-3740 and verify against `0x29B1` before touching the wire — a library whose `crc16_ccitt()` returns `0x2189` for that input is the wrong variant, and every packet it produces will be rejected.

## Framing / resync behaviour

The parser doesn't assume the stream is aligned. Each attempt runs three checks in order — sync bytes, then LEN and a known TYPE, then the CRC — and on any failure drops exactly one byte and retries from the next position. A single corrupted byte (line noise, a mid-packet USB hiccup) therefore costs at most one packet's worth of resync time instead of desyncing the rest of the session. The header check alone is not sufficient, since `0xAA 0x55` can occur inside real payload data by coincidence; LEN, TYPE and CRC are what reject a false header match.

The same three-check sequence runs on both the Pico ([firmware/pico_protocol.c](firmware/pico_protocol.c)) and Pi ([python/pico_protocol.py](python/pico_protocol.py)) sides. They differ in what they accept: the Pico parser accepts all three packet types, the Pi parser accepts only TELEMETRY.

## Structure

```
pico-protocol/
├── README.md
├── TODO.md                     known limitations
├── python/
│   ├── pico_protocol.py        Pi-side: CRC, CMD encoder, TELEMETRY decoder, resync parser
│   └── example_roundtrip.py    encode → decode sanity check, no hardware needed
└── firmware/
    ├── pico_protocol.h         packet structs, constants, CRC, wire-layout assertions
    ├── pico_protocol_rx.h      public API for the receive ring buffer + parser
    ├── pico_protocol.c         ring buffer + resync-safe parser
    └── test_pico_protocol.c    host-side test harness — plain gcc, no hardware
```

## Usage (Python / Pi side)

```python
import serial
from pico_protocol import build_command_packet, read_packet

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
buf = bytearray()

# Send a command
pkt = build_command_packet(
    pose={'x': 0, 'y': 0, 'z': -1.2, 'roll': 0, 'pitch': 0, 'yaw': 45},
    target={'x': 1, 'y': 0, 'z': -1.5, 'roll': 0, 'pitch': 0, 'yaw': 45},
    armed=1,
)
ser.write(pkt)

# Receive telemetry
buf.extend(ser.read(ser.in_waiting or 1))
telemetry = read_packet(buf)   # None until a full valid packet has arrived
if telemetry:
    print(telemetry['raw_depth_m'], telemetry['esc_pwm'])
```

Run `python3 example_roundtrip.py` for a self-contained test that doesn't need a Pico attached.

## Usage (firmware / Pico side)

Feed received bytes in from wherever your UART or USB-CDC RX happens — ISR, DMA, or a polling loop — and pull whole packets back out:

```c
pp_rx_init();                       // once, at startup

pp_rx_write(bytes, n);              // from your RX source

uint8_t type, payload[PAYLOAD_LEN];
while (pp_rx_try_parse(&type, payload)) {
    if (type == TYPE_CMD) {
        const CommandPayload *cmd = (const CommandPayload *)payload;
        // ... feed cmd into the control loop
    }
}
```

`pp_rx_write()` returns how many bytes it accepted, so a full buffer is detectable rather than silent. Build and run the test harness on a laptop:

```bash
gcc -Wall -Wextra -o test_pico_protocol firmware/test_pico_protocol.c firmware/pico_protocol.c && ./test_pico_protocol
```

## Attribution and scope

The C firmware implementation — packet structs, CRC, ring buffer, and the resync-safe parser — is my own work. The Python reference implementation was AI-generated and then validated against the C side: the two ends' field offsets are asserted against each other, and the CRC was verified against the published CRC-16/IBM-3740 check value.

## Known limitations

Full list in [TODO.md](TODO.md). The one worth stating up front:

**There is no protocol version field.** The header carries `STX1`, `STX2`, `LEN`, and `TYPE` — nothing identifies which revision of a payload layout a packet was built against. Because `TYPE` and `LEN` are unchanged when a payload's internal layout changes, an old Pi-side parser talking to new firmware sees valid sync bytes, a matching `LEN`, a known `TYPE`, and a CRC that is genuinely correct over the new bytes. It accepts the packet and decodes the fields to wrong values, silently — no error, no log line, no CRC failure. On a vehicle that surfaces as inexplicable control behaviour rather than as a link fault.

The mitigation, unimplemented, is to spend one of the reserved payload bytes on a version number and reject unknown versions at the parser. That is itself a breaking change, so it wants to happen at a deliberate cut rather than incrementally.

## Why a custom protocol instead of something off-the-shelf

Fixed-size, statically-typed packed structs on both ends means no runtime parsing or allocation cost in the firmware's 50 Hz control loop, and the byte-for-byte layout is identical to the C struct it came from — no serialization library, no schema negotiation, nothing that can fail in a way that isn't a straight CRC mismatch.
