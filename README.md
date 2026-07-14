# pico-protocol

A binary serial packet protocol for a Raspberry Pi <-> RP2350 (Pico 2) link, built for real-time control of an ROV/AUV thruster system. Built standalone, later integrated into [Team Tiburon](#)'s AUV firmware for SAUVC 2026.

Handles both directions of a 115200-baud link between a Pi (mission computer, running fused navigation state) and a Pico (running the real-time control loop and driving 8 ESCs), plus CRC-validated framing that self-recovers from a corrupted or misaligned byte stream without stalling.

## Wire format

```
[ STX1 ][ STX2 ][ LEN ][ TYPE ][ ... PAYLOAD (56 bytes) ... ][ CRC_HI ][ CRC_LO ]
  0xAA    0x55    1B     1B              56 bytes                1B       1B
```

- **STX1 / STX2** — fixed 2-byte sync sequence (`0xAA 0x55`)
- **LEN** — payload length in bytes (56 for every packet type below)
- **TYPE** — packet type, see table below
- **PAYLOAD** — type-specific struct, little-endian, always 56 bytes
- **CRC** — CRC-16-CCITT (polynomial `0x1021`, initial value `0xFFFF`), computed over `[LEN, TYPE, PAYLOAD]` — **not** over the sync bytes

Total packet size: `4 + 56 + 2 = 62 bytes`.

## Packet types

| Type | Direction | Purpose |
|------|-----------|---------|
| `0x01` TELEMETRY | Pico → Pi | Status snapshot: depth, PID outputs, ESC PWM values, armed/link/saturation flags |
| `0x02` CMD | Pi → Pico | Fused current pose + target pose (6-DOF) + arm state + sequence number |
| `0x03` PID | Pi → Pico | Live PID gain updates (kp/ki or kd/kff), so gains can be retuned without reflashing |

`0x01` and `0x02` are implemented on both sides in this repo. `0x03` is defined and consumed on the firmware side (`firmware/pico_protocol.h`) but doesn't have a Python-side encoder here yet.

## Framing / resync behaviour

The parser doesn't assume the stream is aligned. If the sync bytes are missing, the length/type don't match a known packet, or the CRC fails, it drops exactly one byte and retries from there — so a single corrupted byte (line noise, a mid-packet USB hiccup) costs at most one packet's worth of resync time instead of desyncing the whole stream. Identical logic on both the Pico (`.ino`) and Pi (`.py`) sides.

## Structure

```
pico-protocol/
├── python/
│   ├── pico_protocol.py       Pi-side: CRC, packet builder, packet parser
│   └── example_roundtrip.py   encode → decode sanity check, no hardware needed
└── firmware/
    └── pico_protocol.h        Pico-side: matching structs, CRC, constants (C)
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

`firmware/pico_protocol.h` defines the matching structs (`TelemetryPayload`, `CommandPayload`, `PidPayload`), constants, and CRC function used by the Pico-side control loop to build and parse the same packets.

## Why a custom protocol instead of something off-the-shelf

Fixed-size, statically-typed packed structs on both ends means no runtime parsing/allocation cost in the firmware's 50Hz control loop, and the byte-for-byte layout is identical to the C struct it came from — no serialization library, no schema negotiation, nothing that can fail in a way that isn't a straight CRC mismatch.
