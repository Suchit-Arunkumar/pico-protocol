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

## Branch coverage is unmeasured

The suite has never been run under `gcov`. Replaying all five tests' byte
streams through a faithful simulation of the parser shows four paths that no
test reaches — the sync-byte reject, CRC reject, and accept branches are hit,
and these are not:

| Branch | Hits |
|---|---|
| `LEN != PAYLOAD_LEN` or unknown `TYPE` reject | **0** |
| ring buffer wrap (`rx_peek` / `rx_eat` modulo) | **0** |
| buffer-full short write in `pp_rx_write()` | **0** |
| a packet split across two `pp_rx_write()` calls | **0** |

The second check in the parser — the one that rejects a bad `LEN` or an unknown
`TYPE` — is never exercised as a *rejection* by any test. Test 4 looks like it
should: it plants `0xAA 0x55` inside a payload. But the parser locks onto the
real header at offset 0 and consumes the whole packet, so it never examines the
planted bytes at all. The test passes for a reason unrelated to what it claims
to prove.

The wrap case matters most of the three remaining. Every test writes at most 124
bytes into a 256-byte buffer after `pp_rx_init()`, so the modulo arithmetic that
makes the buffer circular has never actually been made to wrap. On hardware it
wraps within the first few seconds.

The split-packet case is the normal condition on a real UART — bytes arrive in
whatever chunks the driver hands over, not in whole packets — and no test covers
it.

These numbers come from simulation, not from gcov on the real binary, so treat
them as a strong indication rather than a measurement. Either way, "the resync
logic is proven correct" is an overstatement of what these five tests establish.

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
