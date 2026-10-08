# Known limitations

What this library does not yet guarantee. Each item is a known gap, not a
known defect: nothing listed here is believed to cause incorrect behaviour on
the wire today.

## Single link per image; not interrupt-safe as shipped

The receive ring buffer is file-scope state in `pico_protocol.c`, so one
firmware image can parse one link. The head and tail indices are plain
`size_t`, not atomics. Calling `pp_rx_write()` from an ISR while
`pp_rx_try_parse()` runs in the main loop is the natural single-producer /
single-consumer pattern, but without atomic indices and the matching memory
ordering it is not formally safe. The intended use today is to call both
from the same context (drain the UART, then parse, in the main loop). A
context-struct API with C11 atomics would remove both restrictions.

## Fuzzing is short and single-instance

`firmware/fuzz_rx.c` is a libFuzzer harness over `pp_rx_write()` /
`pp_rx_try_parse()` under ASan and UBSan. Besides memory safety it checks that
every accepted packet has a known TYPE and, re-framed, appears contiguously in
the input. It found nothing in about 3 million runs locally, and CI runs it for
60 s per push from a three-file seed corpus of valid packets. Disabling the CRC
comparison makes it fail within seconds, so the contract check is live. It has
not had a long (hours) campaign or a persistent corpus.

## Coverage is not measured

Branch coverage was reasoned about test by test, and a mutation check
(disabling the CRC comparison) confirms the resync tests fail when they
should. The suite has not been run under `gcov` / `llvm-cov`.

## Version policy is left to the application

Every payload carries a version byte and `pp_packet_version()` reads it, but
the parser does not reject unknown versions. Whether a mismatch should drop
the packet, raise a fault, or be logged and accepted depends on the vehicle's
failure-mode design, so that decision belongs to the firmware using this
library.

## `PID_GAIN_MAX` is a placeholder

The default bound of ±1000 rejects NaN, infinities and absurd magnitudes but
is not tuned to any vehicle. Define `PID_GAIN_MAX` before including the
header to set a vehicle-specific limit.

## Pi side decodes telemetry only

`python/pico_protocol.py` builds CMD and PID packets and decodes TELEMETRY.
It has no streaming parser object; `read_packet()` operates on a
caller-owned `bytearray`.

## Vehicle-specific packet types are not upstreamed

The AUV firmware this protocol was built for adds a TYPE `0x04` IMU packet
(raw BNO055 register block) that is not defined here.

## Host-side testing only

All automated tests run on a host. There is no hardware-in-the-loop test
against a physical RP2350 over USB CDC or UART.
