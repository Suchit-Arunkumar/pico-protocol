# Known limitations

Gaps I know about in this protocol layer. Things that are true, not things that
are hidden. Nothing here is currently believed to be broken on the wire — these
are missing guarantees, unmeasured claims, and contracts that rest on convention
rather than enforcement.

---

## Senders must zero the payload struct — mitigated, not closed

`pico_protocol.h` now provides `pp_telemetry_init()` / `pp_command_init()` /
`pp_pid_init()`, each returning a fully zeroed struct (verified by Test 10 in
`test_pico_protocol.c` — every byte, including the reserved ones, is checked
against an all-zero buffer). The fix for *this repo* is to declare with the
helper instead of `Struct t; t.field = ...`:

```c
TelemetryPayload t = pp_telemetry_init();
t.depth_m = ...;
```

This does not close the underlying gap, it only makes the correct pattern a
one-line call instead of a remembered `memset`. **The actual firmware TX path
lives outside this repo and has not been checked** — nothing here can confirm
the AUV firmware's telemetry-send code actually calls the helper. That
verification is still open.

Not done: rejecting non-zero reserved bytes at the receiver (strict, and
spends the field's forward-compatibility value) — left alone since the
version-byte work below will want to claim some of that same reserved space.

## No protocol version field

Covered in the README. There is no version byte, so a payload layout change that
keeps `TYPE` and `LEN` intact produces packets an older parser accepts as valid —
correct CRC over the new bytes, fields decoded to wrong values, no error raised.

## The parser has never been fuzzed

The test harness exercises hand-written scenarios: a clean packet, leading
noise, a dropped byte, a false `0xAA 0x55` inside a payload, two back-to-back
packets. Every one is a case somebody thought of in advance.

Nothing has fed the parser random or adversarial bytes — no random streams, no
bit flips at every offset, no truncated packets at every possible boundary, no
sustained garbage with valid packets buried at random positions. For a parser
whose entire purpose is surviving a hostile byte stream, "passes the cases I
imagined" is a weaker claim than it sounds.

The parser is small, self-contained, and has no dynamic allocation, so it is a
natural fit for libFuzzer or AFL++ against `pp_rx_write()` / `pp_rx_try_parse()`.
Not done.

## Parser branch coverage — closed

Tests 6–9 in `test_pico_protocol.c` now cover the four previously-untested
branches (`test_bad_len_and_unknown_type_rejected`, `test_ring_buffer_wrap`,
`test_buffer_full_short_write`, `test_packet_split_across_writes`), and Test 4
was rewritten as `test_false_header_before_valid_packet`. Note the rename:
the old test planted a decoy header *inside* an otherwise-intact first
packet, which the parser never examines before consuming the whole 62 bytes
— it could not fail no matter what false-header handling did. The rewrite
puts the decoy *before* the real packet with a CRC that cannot check out, so
check 3 and the one-byte resync are what have to do the work.

Caveat still open: these are hand-simulated branch counts, not `gcov`/`lcov`
on the compiled binary. Running an actual coverage tool against the 11 tests
now in the harness is unverified — do that before trusting "all branches
covered" as more than an informed guess.

## Ring buffer overflow — closed

`pp_rx_dropped_count()` returns a saturating `uint16_t` of bytes dropped by
`pp_rx_write()` since the last `pp_rx_init()` (saturates at `UINT16_MAX`
rather than wrapping, so a maxed reading still reads as "a lot", not zero).
Covered by Test 8. A caller that never checks `pp_rx_write()`'s return value
can now poll this after the fact.

## PID gains are not bounds-checked — validator added, application still open

`pp_pid_validate()` in `pico_protocol.h` rejects NaN, ±Inf, and anything past
`PID_GAIN_MAX` (currently ±1000.0f — a generous placeholder, not a vehicle-
tuned limit) in any of the 12 gain slots, while still accepting
`PID_NO_CHANGE`. Covered by Test 11.

This is a pure check, callable but **not yet called**: the code that
receives a `PidPayload` and applies it to the live control loop lives in the
AUV firmware, outside this repo, and has not been updated to call
`pp_pid_validate()` before applying. Wiring that call in is the remaining
step, and it lives in the other repo.

## The two-page gain update is not atomic

A full PID gain set spans two packets: page 0 carries kp and ki, page 1 carries
kd and kff. They arrive as separate frames, so between the two the control loop
runs with kp/ki from the new tuning and kd/kff from the old one.

At 50 Hz that window is at least one control cycle, longer if page 1 is delayed,
and unbounded if page 1 is lost entirely — in which case the mismatched set
stays installed with nothing detecting it. The `PID_NO_CHANGE` sentinel lets a
sender leave individual gains alone, but there is no mechanism to apply both
pages as one transaction. A sequence number covering the pair, with
double-buffered gains swapped only once both pages have landed, would close it.

## CI — closed

`.github/workflows/ci.yml` builds the test harness with `-Wall -Wextra
-Werror` and runs it, and separately imports the Python module (triggering
its `_require()` wire-layout checks) and runs `example_roundtrip.py`. Both
jobs run on every push and PR.
