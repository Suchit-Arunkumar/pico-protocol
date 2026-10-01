# pico-protocol

[![CI](https://github.com/Suchit-Arunkumar/pico-protocol/actions/workflows/ci.yml/badge.svg)](https://github.com/Suchit-Arunkumar/pico-protocol/actions/workflows/ci.yml)

A fixed-frame binary serial protocol for the link between a Raspberry Pi (navigation and state estimation) and an RP2350 / Pico 2 microcontroller (50 Hz thruster control), built for Team Tiburon's autonomous underwater vehicle at NIT Rourkela for SAUVC 2026.

The firmware side is portable C11 with no allocation and no SDK dependencies; the Pi side is dependency-free Python. Both implementations are verified to produce byte-identical packets.

## Highlights

- **Self-resynchronising parser.** Recovers from dropped, corrupted, or misaligned bytes within one packet width. Validated against 2,000 packets buried in noise biased toward sync and header bytes and delivered in random chunk sizes.
- **Layout pinned at compile time and import time.** Every field offset of every payload is asserted with `_Static_assert` in C and checked against the same values when the Python module loads. A field reorder that keeps the size unchanged — which would still pass the CRC and silently decode to wrong values — fails the build.
- **Cross-language golden vectors.** The same reference packets are checked by the C test suite (built from C structs) and the Python test suite (built from the Python encoder).
- **Control-safety helpers.** NaN/Inf/range validation for live PID gain updates, and a transaction layer that applies two-packet gain updates atomically, so the controller never runs on a half-updated gain set.
- **Strict CI.** `-std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`, built with both GCC and Clang, run under AddressSanitizer and UndefinedBehaviorSanitizer.

## Wire format

```
 0      1      2      3      4 .............. 59     60       61
[0xAA] [0x55] [LEN ] [TYPE] [ PAYLOAD 56 B  ] [CRC_HI] [CRC_LO]
                      └──────── CRC-16 covers LEN, TYPE, PAYLOAD ──┘
```

| Field | Size | Notes |
|---|---|---|
| Sync | 2 B | `0xAA 0x55` |
| LEN | 1 B | Always `56` |
| TYPE | 1 B | See below |
| Payload | 56 B | Packed struct, little-endian |
| CRC | 2 B | CRC-16/IBM-3740, big-endian on the wire |

| TYPE | Direction | Contents |
|---|---|---|
| `0x01` TELEMETRY | Pico → Pi | Fused and raw depth, per-DOF PID output, 8 ESC pulse widths, armed / link / saturation flags |
| `0x02` CMD | Pi → Pico | Fused 6-DOF pose, 6-DOF setpoint, arm state, sequence number |
| `0x03` PID | Pi → Pico | Live gain update: page 0 = kp/ki, page 1 = kd/kff, per DOF, with a no-change sentinel |

Every payload carries a protocol version byte. Exact layouts, with offsets, are in [`firmware/pico_protocol.h`](firmware/pico_protocol.h).

## Design decisions

**Fixed-size frames over a serialization library.** The control loop runs at 50 Hz on a microcontroller. Fixed 62-byte frames mean no dynamic allocation, no variable-length parsing, and a receive path whose worst-case cost is known in advance. The trade-off — every packet type must fit 56 bytes — is acceptable for a link with three message types.

**One-byte resync on every failure.** The parser never assumes the front of its buffer is a packet boundary. It checks the sync pair, then LEN and TYPE, then the CRC, and on any failure discards exactly one byte and starts over. The sync pair alone is not enough: `0xAA 0x55` appears naturally inside IEEE-754 float payloads, so the header and CRC checks are what reject false locks.

**CRC-16/IBM-3740, named precisely.** "CRC-16-CCITT" refers to at least two incompatible variants. This link uses IBM-3740 (poly `0x1021`, init `0xFFFF`, no reflection, no final XOR; check value `0x29B1` for `"123456789"`), not KERMIT (check value `0x2189`). Both test suites assert the check value.

**Reserved bytes are part of the contract.** Unused payload bytes are covered by the CRC, so a sender that leaves them uninitialised transmits stack memory, produces valid-looking packets, and makes its output non-deterministic. All payloads are constructed through `pp_*_init()` helpers that zero them.

**Versioning without a flag day.** The version byte occupies a byte that earlier revisions defined as reserved and ignored on receipt, so existing receivers keep working unchanged. What to do with an unknown version is left to the application rather than hard-coded into the parser.

**Atomic gain updates.** A full PID retune spans two packets. Applied on arrival, the controller would run new kp/ki against old kd/kff for at least one control tick — indefinitely if the second packet is lost. Both pages carry a shared transaction ID, and `pp_pid_txn_apply_page()` reports completion only once both have arrived.

## Verification

```bash
make test        # C suite (sanitizers on) + Python suite + round-trip example
```

| Suite | What it covers |
|---|---|
| C — 15 tests, 70 checks | Clean parse; leading noise; dropped byte mid-packet; decoy header with bad CRC; back-to-back packets; bad LEN / unknown TYPE; ring-buffer index wraparound; buffer-full short write and drop counter; packets split across writes at header, payload, and CRC boundaries; init helpers; PID validation; version round-trip; atomic PID transactions; golden vectors and CRC check value; randomized noise stress |
| Python — 9 tests | CRC check value; golden vectors; PID encoder paging and transaction IDs; telemetry resync, CRC rejection, and partial-buffer handling |
| Compile time | Size and offset of every payload field, in both languages |

The resync tests were mutation-checked: disabling the CRC comparison in the parser causes three independent tests to fail.

## Usage

### Firmware (C)

```c
#include "pico_protocol.h"
#include "pico_protocol_rx.h"

pp_rx_init();

// In the main loop: feed whatever bytes the UART / USB CDC produced.
pp_rx_write(rx_bytes, n);

uint8_t type, payload[PAYLOAD_LEN];
static PidTransaction txn;   // pp_pid_txn_init(&txn) once at startup

while (pp_rx_try_parse(&type, payload)) {
    if (type == TYPE_CMD) {
        const CommandPayload *cmd = (const CommandPayload *)payload;
        /* update setpoint from cmd */
    } else if (type == TYPE_PID) {
        const PidPayload *p = (const PidPayload *)payload;
        if (pp_pid_validate(p) &&
            pp_pid_txn_apply_page(&txn, p) == PP_PID_TXN_COMPLETE) {
            /* copy txn.kp / ki / kd / kff into the controller together */
        }
    }
}

// Transmit telemetry.
TelemetryPayload t = pp_telemetry_init();
t.depth_m = depth;
/* ... */
uint8_t frame[PACKET_SIZE];
pp_frame(TYPE_TELEMETRY, &t, frame);
uart_write(frame, PACKET_SIZE);
```

`pp_rx_write()` returns the number of bytes accepted; `pp_rx_dropped_count()` reports the running total dropped to a full buffer.

### Raspberry Pi (Python)

```python
import serial
from pico_protocol import build_command_packet, build_pid_packets, read_packet

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
buf = bytearray()

ser.write(build_command_packet(
    pose={'x': 0, 'y': 0, 'z': -1.2, 'roll': 0, 'pitch': 0, 'yaw': 45},
    target={'x': 1, 'y': 0, 'z': -1.5, 'roll': 0, 'pitch': 0, 'yaw': 45},
    armed=1,
))

# Full retune: two packets sharing one transaction ID.
for pkt in build_pid_packets(kp=[0, 0, 40, 2, 2, 0], ki=[0, 0, 1, 0, 0, 0],
                             kd=[0, 0, 5, .5, .5, 0], kff=[0] * 6, txn_id=1):
    ser.write(pkt)

buf.extend(ser.read(ser.in_waiting or 1))
telemetry = read_packet(buf)    # None until a complete valid packet arrives
```

## Repository layout

```
firmware/
  pico_protocol.h         payload structs, layout assertions, CRC, framing,
                          init / version / PID-validation / transaction helpers
  pico_protocol_rx.h      receive API
  pico_protocol.c         ring buffer and resynchronising parser
  test_pico_protocol.c    host-side test suite
python/
  pico_protocol.py        Pi-side encoder, decoder, and parser
  test_pico_protocol.py   Pi-side test suite
  example_roundtrip.py    hardware-free end-to-end example
Makefile                  make test
LIMITATIONS.md            known gaps
```

## Limitations

The parser is single-instance and not interrupt-safe as shipped, coverage-guided fuzzing has not been run, and testing is host-side only. Full list in [LIMITATIONS.md](LIMITATIONS.md).

## Authorship

The C protocol core — framing, payload layout, CRC, ring buffer, and resynchronising parser — is my design and implementation. The Python implementation, and parts of the test suite and safety helpers, were developed with AI assistance and are covered by the golden-vector, layout, and stress tests described above.
