// =============================================================================
// pico_protocol.h
// Firmware-side (Pico2/RP2350) definitions for the Pi <-> Pico binary
// packet protocol. Matches python/pico_protocol.py on the Pi side.
//
// Wire format (little-endian):
//   [ STX1 ][ STX2 ][ LEN ][ TYPE ][ ... PAYLOAD (LEN bytes) ... ][ CRC_HI ][ CRC_LO ]
//     0xAA    0x55     1B     1B              56 bytes                 1B       1B
//
//   CRC-16-CCITT (poly 0x1021, init 0xFFFF) computed over [LEN, TYPE, PAYLOAD].
//   Total packet size = 4 + LEN + 2 = 62 bytes.
// =============================================================================

#pragma once
#include <stdint.h>
#include <stddef.h>   // offsetof / size_t, used by the wire-layout assertions
                      // and by crc16()'s size_t parameter below
#include <string.h>   // memcpy, used by packetCRC() below; memset, used by
                      // the pp_*_init() helpers below
#include <stdbool.h>  // bool, used by pp_pid_validate() below
#include <math.h>     // fabsf, used by pp_pid_txn_apply_page() below

// =============================================================================
// FRAMING
// =============================================================================
#define STX1         0xAA
#define STX2         0x55

// PACKET_SIZE is a literal on purpose: it is the on-wire truth, the number a
// logic analyser would show, and it is what every other size is checked
// against below. Deriving it from PAYLOAD_LEN would make those checks vacuous
// -- the arithmetic could not disagree with itself. Kept independent, a struct
// that grows a field now fails the build instead of silently reframing the link.
#define HEADER_SIZE  4                       // STX1 STX2 LEN TYPE
#define PAYLOAD_LEN  56
#define CRC_SIZE     2                       // CRC_HI CRC_LO
#define PACKET_SIZE  62                      // total bytes on the wire

_Static_assert(HEADER_SIZE + PAYLOAD_LEN + CRC_SIZE == PACKET_SIZE,
               "framing constants do not add up to the on-wire packet size");

#define TYPE_TELEMETRY  0x01   // Pico -> Pi : 56-byte telemetry snapshot
#define TYPE_CMD        0x02   // Pi  -> Pico: fused nav state
#define TYPE_PID        0x03   // Pi  -> Pico: live PID gain update

// =============================================================================
// PROTOCOL VERSION
//
// Spends the first reserved byte of every payload on a version number, per
// the mitigation TODO.md and the README named but never implemented. This is
// deliberately NOT a hard break: every reserved byte was already contractually
// "sender picks, receiver ignores" (see the zero-init contract below), so an
// old receiver that still just checks STX/LEN/TYPE/CRC and otherwise ignores
// reserved[0] keeps working unmodified against a new sender. What's new is
// that a receiver MAY look at this byte and reject a version it doesn't
// understand -- that policy is deliberately left to the caller (see
// pp_packet_version() below), not forced here, since this library doesn't
// own the firmware's failure-mode decisions.
// =============================================================================
#define PROTOCOL_VERSION  1

// =============================================================================
// PAYLOAD STRUCTS  (all __packed__, all exactly 56 bytes)
// =============================================================================

// TYPE 0x01 — TELEMETRY (Pico -> Pi)
typedef struct __attribute__((packed)) {
    float    depth_m;          // control depth: current_z from fused Pi state
                                // (echoed back -- Pi should NOT feed this
                                // back into its own estimator)
    float    raw_depth_m;      // raw pressure-sensor depth (debug/telemetry only)
    float    pid_u[6];         // PID output per DOF: surge, sway, heave, roll, pitch, yaw
    uint16_t esc_pwm[8];       // actual ESC PWM values T1-T8 (microseconds)
    uint8_t  armed;            // armed state (0/1)
    uint8_t  sat_flags;        // bit0=sat_vert, bit1=sat_horiz, bit2=sat_yaw
    uint8_t  link_ok;          // 1 = link healthy, 0 = lost/timeout
    uint8_t  version;          // byte 51. PROTOCOL_VERSION of the sender --
                                // was reserved[0]; see PROTOCOL VERSION above.
    uint8_t  reserved[4];      // bytes 52-55. MUST be zeroed by the sender --
                               // see "zero-initialisation" note below.
} TelemetryPayload;
// --- Wire layout, TYPE 0x01 -------------------------------------------------
// Total size AND every field offset are pinned here. A struct that still adds
// up to 56 bytes but has shifted a field (reordered members, a type widened,
// padding sneaking in on a different toolchain) is silently wire-incompatible
// with the Pi side; these catch that at compile time instead of in the water.
_Static_assert(HEADER_SIZE + sizeof(TelemetryPayload) + CRC_SIZE == PACKET_SIZE,
               "TelemetryPayload does not fill a 62-byte packet");
_Static_assert(offsetof(TelemetryPayload, depth_m)     ==  0, "telemetry.depth_m moved");
_Static_assert(offsetof(TelemetryPayload, raw_depth_m) ==  4, "telemetry.raw_depth_m moved");
_Static_assert(offsetof(TelemetryPayload, pid_u)       ==  8, "telemetry.pid_u moved");
_Static_assert(offsetof(TelemetryPayload, esc_pwm)     == 32, "telemetry.esc_pwm moved");
_Static_assert(offsetof(TelemetryPayload, armed)       == 48, "telemetry.armed moved");
_Static_assert(offsetof(TelemetryPayload, sat_flags)   == 49, "telemetry.sat_flags moved");
_Static_assert(offsetof(TelemetryPayload, link_ok)     == 50, "telemetry.link_ok moved");
_Static_assert(offsetof(TelemetryPayload, version)     == 51, "telemetry.version moved");
_Static_assert(offsetof(TelemetryPayload, reserved)    == 52, "telemetry.reserved moved");

// TYPE 0x02 — CMD (Pi -> Pico)
typedef struct __attribute__((packed)) {
    float    current_x, current_y, current_z;
    float    current_roll, current_pitch, current_yaw;
    float    target_x, target_y, target_z;
    float    target_roll, target_pitch, target_yaw;
    uint8_t  armed;
    uint8_t  seq;
    uint8_t  version;          // byte 50. PROTOCOL_VERSION of the sender --
                                // was reserved[0]; see PROTOCOL VERSION above.
    uint8_t  reserved[5];      // bytes 51-55. MUST be zeroed by the sender --
                               // see "zero-initialisation" note below.
} CommandPayload;
// --- Wire layout, TYPE 0x02 -------------------------------------------------
_Static_assert(HEADER_SIZE + sizeof(CommandPayload) + CRC_SIZE == PACKET_SIZE,
               "CommandPayload does not fill a 62-byte packet");
_Static_assert(offsetof(CommandPayload, current_x)     ==  0, "cmd.current_x moved");
_Static_assert(offsetof(CommandPayload, current_y)     ==  4, "cmd.current_y moved");
_Static_assert(offsetof(CommandPayload, current_z)     ==  8, "cmd.current_z moved");
_Static_assert(offsetof(CommandPayload, current_roll)  == 12, "cmd.current_roll moved");
_Static_assert(offsetof(CommandPayload, current_pitch) == 16, "cmd.current_pitch moved");
_Static_assert(offsetof(CommandPayload, current_yaw)   == 20, "cmd.current_yaw moved");
_Static_assert(offsetof(CommandPayload, target_x)      == 24, "cmd.target_x moved");
_Static_assert(offsetof(CommandPayload, target_y)      == 28, "cmd.target_y moved");
_Static_assert(offsetof(CommandPayload, target_z)      == 32, "cmd.target_z moved");
_Static_assert(offsetof(CommandPayload, target_roll)   == 36, "cmd.target_roll moved");
_Static_assert(offsetof(CommandPayload, target_pitch)  == 40, "cmd.target_pitch moved");
_Static_assert(offsetof(CommandPayload, target_yaw)    == 44, "cmd.target_yaw moved");
_Static_assert(offsetof(CommandPayload, armed)         == 48, "cmd.armed moved");
_Static_assert(offsetof(CommandPayload, seq)           == 49, "cmd.seq moved");
_Static_assert(offsetof(CommandPayload, version)       == 50, "cmd.version moved");
_Static_assert(offsetof(CommandPayload, reserved)      == 51, "cmd.reserved moved");

// TYPE 0x03 — PID tuning (Pi -> Pico)
// page 0 -> gains_a = kp[6],  gains_b = ki[6]
// page 1 -> gains_a = kd[6],  gains_b = kff[6]
// Sentinel PID_NO_CHANGE (-999.0f) on any element means "leave unchanged".
//
// txn_id groups the two pages of one logical gain update. A sender doing a
// full-gains retune sends both pages with the SAME txn_id; pp_pid_transaction_*
// below (see ATOMIC TWO-PAGE UPDATE) uses it to apply both pages together
// instead of letting the control loop run for a tick on a mismatched half-set.
// A sender only ever touching one page (one DOF's kp, say) can leave txn_id
// at 0 -- nothing requires pairing when there's nothing to pair.
typedef struct __attribute__((packed)) {
    uint8_t  page;
    uint8_t  version;          // byte 1. PROTOCOL_VERSION of the sender --
                                // was reserved0[0]; see PROTOCOL VERSION above.
    uint8_t  txn_id;           // byte 2. Pairs page 0 and page 1 of one
                                // update. Was reserved0[1]; see ATOMIC
                                // TWO-PAGE UPDATE below.
    uint8_t  reserved0;        // byte 3. Was reserved0[2].
    float    gains_a[6];
    float    gains_b[6];
    uint8_t  tail[4];          // bytes 52-55. Reserved for expansion; zero on send,
                               // ignore on receive. Covered by the CRC.
} PidPayload;
// --- Wire layout, TYPE 0x03 -------------------------------------------------
_Static_assert(HEADER_SIZE + sizeof(PidPayload) + CRC_SIZE == PACKET_SIZE,
               "PidPayload does not fill a 62-byte packet");
_Static_assert(offsetof(PidPayload, page)      ==  0, "pid.page moved");
_Static_assert(offsetof(PidPayload, version)   ==  1, "pid.version moved");
_Static_assert(offsetof(PidPayload, txn_id)    ==  2, "pid.txn_id moved");
_Static_assert(offsetof(PidPayload, reserved0) ==  3, "pid.reserved0 moved");
_Static_assert(offsetof(PidPayload, gains_a)   ==  4, "pid.gains_a moved");
_Static_assert(offsetof(PidPayload, gains_b)   == 28, "pid.gains_b moved");
_Static_assert(offsetof(PidPayload, tail)      == 52, "pid.tail moved");

// Returns the version byte of a decoded payload, given the packet's TYPE
// (pp_rx_try_parse()'s out_type). Policy on what to do with a version this
// receiver doesn't recognise -- reject, log-and-accept, etc. -- is the
// caller's to make; this just reads the byte so that decision has data to
// act on. Returns 0 (never a valid PROTOCOL_VERSION) for an unrecognised type.
static inline uint8_t pp_packet_version(uint8_t type, const uint8_t *payload) {
    switch (type) {
        case TYPE_TELEMETRY: return ((const TelemetryPayload *)payload)->version;
        case TYPE_CMD:        return ((const CommandPayload  *)payload)->version;
        case TYPE_PID:        return ((const PidPayload      *)payload)->version;
        default:              return 0;
    }
}

#define PID_NO_CHANGE  -999.0f

// =============================================================================
// ZERO-INITIALISATION CONTRACT
//
// Every one of the 56 payload bytes is a named field -- there is no
// compiler-inserted padding in these packed structs. The trailing reserved[]
// / tail[] bytes are therefore real bytes that the sender chooses, and the
// CRC is computed over them.
//
// Skipping zero-init does NOT produce a detectable error. Stack garbage in
// the reserved bytes is covered by the CRC, so the packet is well-formed and
// the receiver accepts it. The costs are silent: uninitialised stack memory
// is transmitted on every frame, and two packets with identical logical
// content no longer produce identical bytes -- which makes golden-vector
// tests (compare built packet against known-good bytes) impossible to write.
//
// Use the pp_*_init() helpers below instead of declaring-then-assigning:
// they are the only sanctioned way to get a payload struct into a
// zero-reserved-bytes state, so "did the sender memset?" becomes "did the
// sender call the helper?" -- a much easier thing to grep for and review.
//
//     TelemetryPayload t = pp_telemetry_init();
//     t.depth_m = ...;
//
// The firmware TX path lives outside this repo and is NOT verified here --
// these helpers reduce the chance of the mistake, they cannot prevent a
// caller from ignoring them.
// =============================================================================

static inline TelemetryPayload pp_telemetry_init(void) {
    TelemetryPayload t;
    memset(&t, 0, sizeof(t));
    t.version = PROTOCOL_VERSION;
    return t;
}

static inline CommandPayload pp_command_init(void) {
    CommandPayload c;
    memset(&c, 0, sizeof(c));
    c.version = PROTOCOL_VERSION;
    return c;
}

static inline PidPayload pp_pid_init(void) {
    PidPayload p;
    memset(&p, 0, sizeof(p));
    p.version = PROTOCOL_VERSION;
    return p;
}

// =============================================================================
// CRC-16-CCITT
// =============================================================================

static inline uint16_t crc16(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int j = 0; j < 8; j++)
            c = (c & 0x8000) ? ((c << 1) ^ 0x1021) : (c << 1);
    }
    return c;
}

// len_field is the LEN *header byte*, hashed as data -- it is NOT the length of
// `payload`. The CRC always covers exactly PAYLOAD_LEN payload bytes, because
// every packet type on this link is fixed at 56. Passing anything other than
// PAYLOAD_LEN produces a CRC over a LEN byte that contradicts the bytes
// actually hashed, which the receiver will reject as corruption.
static inline uint16_t packetCRC(uint8_t len_field, uint8_t type, const uint8_t *payload) {
    uint8_t ci[2 + PAYLOAD_LEN];
    ci[0] = len_field; ci[1] = type;
    memcpy(&ci[2], payload, PAYLOAD_LEN);
    return crc16(ci, sizeof(ci));
}

// =============================================================================
// PID GAIN VALIDATION
//
// The CRC proves a PidPayload arrived intact; it says nothing about whether
// the gain values inside it were ever sane. A corrupted-but-CRC-valid
// packet, or a bug on the Pi side, can put NaN, +-Inf, or an absurd
// magnitude into gains_a/gains_b, and nothing downstream currently rejects
// that before it reaches a live control loop.
//
// This is a pure, allocation-free check the firmware is expected to call
// before applying a received PidPayload -- it does not apply anything
// itself (that logic lives in the AUV firmware, outside this repo).
// PID_NO_CHANGE (-999.0f) is always accepted regardless of the bounds below,
// since it means "leave this gain alone" rather than "set it to -999".
// =============================================================================
#define PID_GAIN_MAX  1000.0f   // generous upper bound for any kp/ki/kd/kff
                                // this link will ever carry; tune to the
                                // vehicle's actual gain range if known.

static inline bool pp_pid_gain_ok(float g) {
    if (g == PID_NO_CHANGE) return true;
    if (g != g) return false;             // NaN: only value that isn't == itself
    if (g < -PID_GAIN_MAX || g > PID_GAIN_MAX) return false;  // also catches +-Inf
    return true;
}

// Validates every gain in both gains_a[6] and gains_b[6]. Returns true only
// if all twelve pass pp_pid_gain_ok(). Callers should discard (not apply)
// a PidPayload for which this returns false.
static inline bool pp_pid_validate(const PidPayload *p) {
    for (int i = 0; i < 6; i++) {
        if (!pp_pid_gain_ok(p->gains_a[i])) return false;
        if (!pp_pid_gain_ok(p->gains_b[i])) return false;
    }
    return true;
}

// =============================================================================
// ATOMIC TWO-PAGE UPDATE
//
// A full PID retune is two packets: page 0 (kp, ki) and page 1 (kd, kff).
// Applying each page the instant it arrives -- what the TODO.md gap
// described, and what the AUV firmware's pollPackets() still does -- means
// the control loop runs with the new kp/ki and the OLD kd/kff for at least
// one tick, longer if page 1 is delayed, and keeps the mismatched set
// installed forever if page 1 is lost.
//
// PidTransaction assembles both pages before anything is applied. The
// caller feeds every received (already-CRC-checked, already-validated)
// PidPayload into pp_pid_txn_apply_page(); the four gain arrays are only
// populated, and PP_PID_TXN_COMPLETE only returned, once both pages for the
// SAME txn_id have landed. The caller applies kp/ki/kd/kff together, in one
// place, exactly once per completed transaction -- there is no tick where a
// half-applied set is live.
//
// This struct, like pp_pid_validate() above, assembles data; it does not
// apply it. Applying to a live control loop is firmware work, outside this
// repo.
// =============================================================================
typedef enum {
    PP_PID_TXN_INCOMPLETE,   // this page landed; waiting on the other one
    PP_PID_TXN_COMPLETE,     // both pages of this txn_id are now in gains_*
    PP_PID_TXN_RESTARTED     // a page arrived whose txn_id didn't match the
                             // one in progress -- the old partial page was
                             // discarded and this page started a new one
} PpPidTxnResult;

typedef struct {
    uint8_t  txn_id;
    bool     have_page0;
    bool     have_page1;
    float    kp[6];
    float    ki[6];
    float    kd[6];
    float    kff[6];
} PidTransaction;

static inline void pp_pid_txn_init(PidTransaction *txn) {
    memset(txn, 0, sizeof(*txn));
}

// Feed one already-validated PidPayload in. Returns what happened; on
// PP_PID_TXN_COMPLETE, txn->kp/ki/kd/kff hold the full, matched set to apply.
// PID_NO_CHANGE entries are left as this transaction's prior value (which is
// 0.0f for a field never touched by either page -- callers wanting "leave
// the vehicle's current gain alone" for an untouched field should seed
// txn->kp/ki/kd/kff from the live gains before calling this, not rely on the
// zero default).
static inline PpPidTxnResult pp_pid_txn_apply_page(PidTransaction *txn, const PidPayload *p) {
    PpPidTxnResult result = PP_PID_TXN_INCOMPLETE;

    bool restarted = (txn->have_page0 || txn->have_page1) && (p->txn_id != txn->txn_id);
    if (restarted) {
        pp_pid_txn_init(txn);
        result = PP_PID_TXN_RESTARTED;
    }
    txn->txn_id = p->txn_id;

    if (p->page == 0) {
        for (int i = 0; i < 6; i++) {
            if (fabsf(p->gains_a[i] - PID_NO_CHANGE) > 0.001f) txn->kp[i] = p->gains_a[i];
            if (fabsf(p->gains_b[i] - PID_NO_CHANGE) > 0.001f) txn->ki[i] = p->gains_b[i];
        }
        txn->have_page0 = true;
    } else if (p->page == 1) {
        for (int i = 0; i < 6; i++) {
            if (fabsf(p->gains_a[i] - PID_NO_CHANGE) > 0.001f) txn->kd[i]  = p->gains_a[i];
            if (fabsf(p->gains_b[i] - PID_NO_CHANGE) > 0.001f) txn->kff[i] = p->gains_b[i];
        }
        txn->have_page1 = true;
    }
    // Unknown page numbers: counted as neither page, forward-compatible no-op.

    if (txn->have_page0 && txn->have_page1) {
        return PP_PID_TXN_COMPLETE;
    }
    return result;
}
