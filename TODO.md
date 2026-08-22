# TODO

Known gaps in this repo. This is the list of things I know about, not a list
of things I think are fine.

---

## Senders must zero the payload struct, and nothing enforces it

Every one of the 56 payload bytes is a named field — there is no
compiler-inserted padding in these packed structs. The trailing `reserved[]`
bytes (`TelemetryPayload.reserved[5]`, bytes 51–55; `CommandPayload.reserved[6]`,
bytes 50–55) are therefore real bytes the sender chooses, and the CRC is
computed over them.

A sender that does `TelemetryPayload t;` on the stack and assigns fields
individually, without `memset`, puts uninitialised stack memory on the wire.

This fails silently in an unusually unhelpful way:

- The CRC is computed *over* the garbage, so it is a valid CRC. The receiver
  accepts the packet. Nothing errors, nothing logs.
- Five bytes of Pico stack memory are transmitted every frame, at 50 Hz.
- Byte-level reproducibility is lost. Two packets with identical logical
  content produce different bytes and different CRCs, which makes
  golden-vector tests — build a packet, compare against known-good bytes —
  impossible to write. That forecloses a whole class of testing.

The contract is documented in `firmware/pico_protocol.h`, and both in-repo
packers honour it (the test harness `memset`s; the Python builder packs
explicit zero bytes). **The actual firmware TX path lives outside this repo and
has not been checked.** That is the thing to go and verify.

Options beyond documenting it: have the receiver reject non-zero `reserved`
bytes (strict, and burns the field's forward-compatibility value), or provide
a `telemetry_init()` helper that senders are expected to call.

---

## `PidPayload.tail[4]` is undocumented

Bytes 52–55 of the TYPE_0x03 payload. Unlike the `reserved[]` fields on the
other two structs, it has no comment saying what it is for, and the name does
not say either — `tail` describes where it sits, not what it means.

Decide which it is:

- **Reserved for expansion** — then rename it `reserved[4]` for consistency
  with the other two payloads, document it, and it falls under the zero-init
  contract above.
- **Vestigial** — an artifact of padding the struct out to 56 bytes, in which
  case say so explicitly in a comment, because a future reader will otherwise
  assume it carries meaning.

Not urgent: there is no Python-side TYPE_PID encoder yet, so nothing currently
writes this field. Worth settling *before* one is written, not after.

---

## `packetCRC()` ignores its own `len` argument

`packetCRC(uint8_t len, uint8_t type, const uint8_t *payload)` writes `len`
into the CRC input as data, then unconditionally copies `PAYLOAD_LEN` bytes
regardless of what `len` says.

Harmless today — every packet type is fixed at 56 bytes, so `len` is always
`PAYLOAD_LEN` at every call site. It becomes a real bug the moment a
variable-length packet type is introduced: the CRC would silently cover 56
bytes no matter what was passed, and the mismatch would present as
unexplainable CRC failures rather than as an obvious length bug.

Either use `len` for the copy and bound-check it against the buffer, or drop
the parameter and read `PAYLOAD_LEN` directly so the signature stops implying
a flexibility that does not exist.
