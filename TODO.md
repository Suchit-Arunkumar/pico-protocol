# Known limitations

Gaps I know about in this protocol layer. Things that are true, not things that
are hidden. Nothing here is currently believed to be broken on the wire — these
are missing guarantees, unmeasured claims, and contracts that rest on convention
rather than enforcement.

---

## Senders must zero the payload struct, and nothing enforces it

Every one of the 56 payload bytes is a named field — there is no
compiler-inserted padding in these packed structs. The trailing reserved bytes
(`TelemetryPayload.reserved[5]` at bytes 51–55, `CommandPayload.reserved[6]` at
50–55, `PidPayload.tail[4]` at 52–55) are real bytes the sender chooses, and the
CRC is computed over them.

A sender that declares `TelemetryPayload t;` on the stack and assigns fields
individually, without `memset`, puts uninitialised stack memory on the wire.

This fails silently, in an unhelpful way:

- The CRC is computed *over* the garbage, so it is a valid CRC. The receiver
  accepts the packet. Nothing errors, nothing logs.
- Bytes of Pico stack memory are transmitted every frame, at 50 Hz.
- Byte-level reproducibility is lost. Two packets with identical logical content
  produce different bytes and different CRCs, which makes golden-vector tests —
  build a packet, compare against known-good bytes — impossible to write. That
  forecloses a whole class of testing.

The contract is documented in `firmware/pico_protocol.h`, and both in-repo
packers honour it: the test harness `memset`s, and the Python builder packs
explicit zero bytes. **The actual firmware TX path lives outside this repo and
has not been checked.** That is the thing to go and verify.

Options beyond documenting it: reject non-zero reserved bytes at the receiver
(strict, and spends the field's forward-compatibility value), or provide an
init helper that senders are expected to call.

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

## Four parser branches have no test — tests to write

Replaying all five existing tests' byte streams through a simulation of the
parser shows the sync-byte reject, CRC reject, and accept branches are hit, and
four paths are never reached at all. These are the tests to write, one per
uncovered branch. (Counts are from simulation, not `gcov` on the real binary —
the suite has never been run under a coverage tool, which is its own gap.)

**`test_bad_len_and_unknown_type_rejected`**
Feed a frame with correct sync bytes and a valid CRC but `LEN != 56`, and a
second with a `TYPE` outside `{0x01, 0x02, 0x03}`. Both must be rejected, and a
valid packet placed after them must still be recovered. This is the parser's
second check, and no current test exercises it as a *rejection*.

**`test_ring_buffer_wrap`**
Write and parse more than `RX_BUF_SIZE` (256) bytes' worth of packets in a
single session, without an intervening `pp_rx_init()`, so `g_head` and `g_tail`
cross the modulo boundary. Every existing test writes at most 124 bytes after
init, so the arithmetic that makes the buffer circular has never actually been
made to wrap. On hardware it wraps within seconds of boot.

**`test_buffer_full_short_write`**
Write more bytes than the buffer can hold and assert `pp_rx_write()` returns
fewer than requested, that the accepted prefix is intact and still parses, and
that the buffer recovers once drained. The short-write return path is the only
overflow signal the API offers and nothing currently checks it.

**`test_packet_split_across_writes`**
Deliver one packet in several `pp_rx_write()` calls — split mid-header,
mid-payload, and between the two CRC bytes — asserting `pp_rx_try_parse()`
returns false until the final chunk lands, then returns the packet intact. This
is the *normal* case on a real UART, where bytes arrive in whatever chunks the
driver hands over rather than in whole packets, and no test covers it.

### Test 4 passes for the wrong reason and needs rewriting

`test_false_header_inside_payload` claims to prove that a coincidental
`0xAA 0x55` inside payload data doesn't cause a false lock. It plants that pair
at `pkt[10..11]` and then asserts the outer packet parses.

It cannot fail. The parser locks onto the real header at offset 0, validates it,
and consumes all 62 bytes — so it never examines the planted bytes at all. The
assertion passes for a reason unrelated to what the test claims to establish,
and would keep passing even if false-header handling were completely broken.

To actually test the intent, the fake header must be reached *before* any real
one: write a run of junk that contains `0xAA 0x55` followed by a plausible-
looking `LEN`/`TYPE` and then bytes whose CRC does not check out, and only
after that append a genuine packet. The parser must reject the decoy on the CRC
check, resync, and recover the real packet. That version exercises the
false-lock path and fails if the CRC check is removed — the current one does
not.

## Ring buffer overflow is detectable but not counted

`pp_rx_write()` returns the number of bytes it accepted, which is fewer than
requested when the buffer is full — so a caller *can* detect overflow. Nothing
records it. There is no counter, no flag, and no way to answer "did we drop
bytes during that run, and how many" after the fact.

A caller that ignores the return value loses bytes silently, and the resulting
symptom is an unexplained CRC failure or a missing packet rather than an
overflow report. A saturating `uint16_t` counter plus a getter would make the
condition observable in telemetry.

## PID gains are not bounds-checked

The protocol layer accepts whatever 32-bit floats arrive in `gains_a` /
`gains_b` and passes them through. There is no range check, no NaN or infinity
rejection, and no sanity limit. A corrupted-but-CRC-valid packet, or a Pi-side
bug, can install a gain of `1e30` or `NaN` into a live control loop.

CRC catches corruption in transit; it does nothing about a value that was wrong
before it was framed. Per-gain bounds belong either at this layer or at the
point of application.

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

## No CI

Nothing builds the firmware, runs the test harness, or imports the Python module
automatically. All verification to date has been run by hand. The test harness
already exits non-zero on failure, so it is ready to drop into a CI step
whenever one exists.
