// =============================================================================
// pico_protocol.h
//
// Wire definitions for the Raspberry Pi <-> Pico 2 (RP2350) binary packet
// protocol: framing constants, payload structs, CRC, and the payload-level
// helpers (initialisation, version read-back, PID gain validation, atomic
// two-page PID assembly). Mirrored byte for byte by python/pico_protocol.py.
//
// Wire format:
//   [ STX1 ][ STX2 ][ LEN ][ TYPE ][ PAYLOAD (56 bytes) ][ CRC_HI ][ CRC_LO ]
//     0xAA    0x55    56     1B      little-endian          big-endian
//
//   CRC-16/IBM-3740 over [LEN, TYPE, PAYLOAD]. 62 bytes total.
//
// Header-only and freestanding apart from <string.h>/<math.h>: no allocation,
// no globals. The receive ring buffer and parser are in pico_protocol.c.
// =============================================================================

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// =============================================================================
// FRAMING
//
// PACKET_SIZE is a literal, not derived from the others. It is the on-wire
// truth -- the number a logic analyser shows -- and every struct size below
// is asserted against it. Deriving it would make those assertions vacuous.
// =============================================================================
#define STX1         0xAA
#define STX2         0x55
#define HEADER_SIZE  4          // STX1 STX2 LEN TYPE
#define PAYLOAD_LEN  56
#define CRC_SIZE     2          // CRC_HI CRC_LO
#define PACKET_SIZE  62

_Static_assert(HEADER_SIZE + PAYLOAD_LEN + CRC_SIZE == PACKET_SIZE,
               "framing constants do not add up to the on-wire packet size");

#define TYPE_TELEMETRY  0x01    // Pico -> Pi
#define TYPE_CMD        0x02    // Pi   -> Pico
#define TYPE_PID        0x03    // Pi   -> Pico

// =============================================================================
// PROTOCOL VERSION
//
// Every payload carries a version byte. Senders stamp it via the pp_*_init()
// helpers; receivers read it with pp_packet_version(). The version occupies a
// byte that earlier revisions defined as reserved-and-ignored, so a receiver
// that never reads it remains compatible. What a receiver does with an
// unrecognised version (reject, log, accept) is a policy decision left to the
// application.
// =============================================================================
#define PROTOCOL_VERSION  1

// =============================================================================
// PAYLOAD STRUCTS
//
// All packed, all exactly PAYLOAD_LEN bytes, all little-endian. Size and every
// field offset are pinned with _Static_assert: a change that preserves total
// size but moves a field (reorder, widened type, toolchain padding) would
// still produce valid-CRC packets that decode to wrong values on the Pi, so it
// must fail the build instead. python/pico_protocol.py asserts the same
// offsets at import time.
// =============================================================================

// ---- TYPE 0x01 : TELEMETRY (Pico -> Pi) -------------------------------------
typedef struct __attribute__((packed)) {
    float    depth_m;          // fused control depth echoed from the Pi's CMD.
                               // The Pi must not feed this back into its own
                               // estimator.
    float    raw_depth_m;      // onboard pressure sensor (telemetry only)
    float    pid_u[6];         // PID output per DOF: surge sway heave roll pitch yaw
    uint16_t esc_pwm[8];       // ESC pulse width per thruster T1..T8, microseconds
    uint8_t  armed;            // 0 / 1
    uint8_t  sat_flags;        // bit0 vertical, bit1 horizontal, bit2 yaw saturated
    uint8_t  link_ok;          // 0 = command timeout / link lost
    uint8_t  version;          // PROTOCOL_VERSION
    uint8_t  reserved[4];      // zero on send
} TelemetryPayload;

_Static_assert(sizeof(TelemetryPayload) == PAYLOAD_LEN, "TelemetryPayload size");
_Static_assert(offsetof(TelemetryPayload, depth_m)     ==  0, "telemetry.depth_m");
_Static_assert(offsetof(TelemetryPayload, raw_depth_m) ==  4, "telemetry.raw_depth_m");
_Static_assert(offsetof(TelemetryPayload, pid_u)       ==  8, "telemetry.pid_u");
_Static_assert(offsetof(TelemetryPayload, esc_pwm)     == 32, "telemetry.esc_pwm");
_Static_assert(offsetof(TelemetryPayload, armed)       == 48, "telemetry.armed");
_Static_assert(offsetof(TelemetryPayload, sat_flags)   == 49, "telemetry.sat_flags");
_Static_assert(offsetof(TelemetryPayload, link_ok)     == 50, "telemetry.link_ok");
_Static_assert(offsetof(TelemetryPayload, version)     == 51, "telemetry.version");
_Static_assert(offsetof(TelemetryPayload, reserved)    == 52, "telemetry.reserved");

// ---- TYPE 0x02 : CMD (Pi -> Pico) -------------------------------------------
typedef struct __attribute__((packed)) {
    float    current_x, current_y, current_z;            // fused pose from the Pi
    float    current_roll, current_pitch, current_yaw;
    float    target_x, target_y, target_z;               // setpoint
    float    target_roll, target_pitch, target_yaw;
    uint8_t  armed;
    uint8_t  seq;                                         // increments per packet
    uint8_t  version;                                     // PROTOCOL_VERSION
    uint8_t  reserved[5];                                 // zero on send
} CommandPayload;

_Static_assert(sizeof(CommandPayload) == PAYLOAD_LEN, "CommandPayload size");
_Static_assert(offsetof(CommandPayload, current_x)     ==  0, "cmd.current_x");
_Static_assert(offsetof(CommandPayload, current_y)     ==  4, "cmd.current_y");
_Static_assert(offsetof(CommandPayload, current_z)     ==  8, "cmd.current_z");
_Static_assert(offsetof(CommandPayload, current_roll)  == 12, "cmd.current_roll");
_Static_assert(offsetof(CommandPayload, current_pitch) == 16, "cmd.current_pitch");
_Static_assert(offsetof(CommandPayload, current_yaw)   == 20, "cmd.current_yaw");
_Static_assert(offsetof(CommandPayload, target_x)      == 24, "cmd.target_x");
_Static_assert(offsetof(CommandPayload, target_y)      == 28, "cmd.target_y");
_Static_assert(offsetof(CommandPayload, target_z)      == 32, "cmd.target_z");
_Static_assert(offsetof(CommandPayload, target_roll)   == 36, "cmd.target_roll");
_Static_assert(offsetof(CommandPayload, target_pitch)  == 40, "cmd.target_pitch");
_Static_assert(offsetof(CommandPayload, target_yaw)    == 44, "cmd.target_yaw");
_Static_assert(offsetof(CommandPayload, armed)         == 48, "cmd.armed");
_Static_assert(offsetof(CommandPayload, seq)           == 49, "cmd.seq");
_Static_assert(offsetof(CommandPayload, version)       == 50, "cmd.version");
_Static_assert(offsetof(CommandPayload, reserved)      == 51, "cmd.reserved");

// ---- TYPE 0x03 : PID gain update (Pi -> Pico) -------------------------------
//   page 0 : gains_a = kp[6], gains_b = ki[6]
//   page 1 : gains_a = kd[6], gains_b = kff[6]
//
// An element equal to PID_NO_CHANGE leaves that gain untouched, so one DOF can
// be retuned without resending the rest. The two pages of a full retune share
// a txn_id so the receiver can apply them as one unit (see ATOMIC TWO-PAGE
// UPDATE below).
typedef struct __attribute__((packed)) {
    uint8_t  page;
    uint8_t  version;          // PROTOCOL_VERSION
    uint8_t  txn_id;           // pairs page 0 and page 1 of one update
    uint8_t  reserved0;        // zero on send
    float    gains_a[6];
    float    gains_b[6];
    uint8_t  tail[4];          // zero on send
} PidPayload;

_Static_assert(sizeof(PidPayload) == PAYLOAD_LEN, "PidPayload size");
_Static_assert(offsetof(PidPayload, page)      ==  0, "pid.page");
_Static_assert(offsetof(PidPayload, version)   ==  1, "pid.version");
_Static_assert(offsetof(PidPayload, txn_id)    ==  2, "pid.txn_id");
_Static_assert(offsetof(PidPayload, reserved0) ==  3, "pid.reserved0");
_Static_assert(offsetof(PidPayload, gains_a)   ==  4, "pid.gains_a");
_Static_assert(offsetof(PidPayload, gains_b)   == 28, "pid.gains_b");
_Static_assert(offsetof(PidPayload, tail)      == 52, "pid.tail");

#define PID_NO_CHANGE  -999.0f

// =============================================================================
// PAYLOAD INITIALISATION
//
// Reserved bytes are real wire bytes and are covered by the CRC. A sender that
// declares a payload on the stack and assigns fields individually transmits
// uninitialised stack memory in them -- the CRC is computed over that memory,
// so the receiver accepts the packet and nothing reports an error. It also
// makes output non-deterministic, which rules out golden-vector testing.
//
// Always construct payloads through these helpers: they zero every byte and
// stamp the version.
//
//     TelemetryPayload t = pp_telemetry_init();
//     t.depth_m = ...;
// =============================================================================
static inline TelemetryPayload pp_telemetry_init(void) {
    TelemetryPayload t;
    memset(&t, 0, sizeof t);
    t.version = PROTOCOL_VERSION;
    return t;
}

static inline CommandPayload pp_command_init(void) {
    CommandPayload c;
    memset(&c, 0, sizeof c);
    c.version = PROTOCOL_VERSION;
    return c;
}

static inline PidPayload pp_pid_init(void) {
    PidPayload p;
    memset(&p, 0, sizeof p);
    p.version = PROTOCOL_VERSION;
    return p;
}

// Version byte of a decoded payload, given the TYPE from pp_rx_try_parse().
// Returns 0 -- never a valid PROTOCOL_VERSION -- for an unknown type.
// Reads the byte at its pinned offset rather than through a struct pointer,
// so the payload buffer needs no particular alignment or effective type.
static inline uint8_t pp_packet_version(uint8_t type, const uint8_t *payload) {
    switch (type) {
        case TYPE_TELEMETRY: return payload[offsetof(TelemetryPayload, version)];
        case TYPE_CMD:       return payload[offsetof(CommandPayload,   version)];
        case TYPE_PID:       return payload[offsetof(PidPayload,       version)];
        default:             return 0;
    }
}

// =============================================================================
// LINK WATCHDOG
//
// The receiver's half of the command-timeout failsafe. Feed it the time of
// every valid CMD packet; pp_link_ok() turns false once PP_LINK_TIMEOUT_MS
// have passed without one, and is false until the first feed. What to do on
// timeout (disarm, neutral thrust) belongs to the firmware.
//
// now_ms is any free-running millisecond counter. The unsigned subtraction
// stays correct across its wrap at 2^32 ms (49.7 days).
// =============================================================================
#ifndef PP_LINK_TIMEOUT_MS
#define PP_LINK_TIMEOUT_MS  500u   // override before including this header
#endif

typedef struct {
    uint32_t last_ms;
    bool     seen;
} PpLinkWatchdog;

static inline void pp_link_init(PpLinkWatchdog *w) {
    w->last_ms = 0;
    w->seen    = false;
}

static inline void pp_link_feed(PpLinkWatchdog *w, uint32_t now_ms) {
    w->last_ms = now_ms;
    w->seen    = true;
}

static inline bool pp_link_ok(const PpLinkWatchdog *w, uint32_t now_ms) {
    return w->seen && (uint32_t)(now_ms - w->last_ms) < PP_LINK_TIMEOUT_MS;
}

// =============================================================================
// CRC-16/IBM-3740  (a.k.a. CRC-16/CCITT-FALSE)
//
// poly 0x1021, init 0xFFFF, no input/output reflection, no final XOR.
// Check value: crc16("123456789") == 0x29B1. This is NOT the KERMIT variant
// (init 0x0000, reflected, check 0x2189) that "CRC-16-CCITT" sometimes means.
// =============================================================================
static inline uint16_t crc16(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c = (uint16_t)(c ^ ((unsigned)d[i] << 8));
        for (int j = 0; j < 8; j++)
            c = (uint16_t)((c & 0x8000u) ? ((unsigned)c << 1) ^ 0x1021u : (unsigned)c << 1);
    }
    return c;
}

// CRC over [LEN, TYPE, PAYLOAD]. `len_field` is the LEN header byte, hashed as
// data; exactly PAYLOAD_LEN payload bytes are always covered.
static inline uint16_t packetCRC(uint8_t len_field, uint8_t type, const uint8_t *payload) {
    uint8_t ci[2 + PAYLOAD_LEN];
    ci[0] = len_field;
    ci[1] = type;
    memcpy(&ci[2], payload, PAYLOAD_LEN);
    return crc16(ci, sizeof ci);
}

// Serialise a payload into a complete 62-byte packet.
static inline void pp_frame(uint8_t type, const void *payload, uint8_t out[PACKET_SIZE]) {
    out[0] = STX1;
    out[1] = STX2;
    out[2] = PAYLOAD_LEN;
    out[3] = type;
    memcpy(&out[HEADER_SIZE], payload, PAYLOAD_LEN);
    uint16_t crc = packetCRC(PAYLOAD_LEN, type, &out[HEADER_SIZE]);
    out[HEADER_SIZE + PAYLOAD_LEN]     = (uint8_t)(crc >> 8);
    out[HEADER_SIZE + PAYLOAD_LEN + 1] = (uint8_t)(crc & 0xFF);
}

// =============================================================================
// PID GAIN VALIDATION
//
// A valid CRC proves the bytes arrived as sent, not that the values were sane
// when sent. Call pp_pid_validate() on every received PidPayload and discard it
// on failure: it rejects NaN, +-Inf, and magnitudes beyond PID_GAIN_MAX in any
// of the twelve gain slots. PID_NO_CHANGE is always accepted.
// =============================================================================
#ifndef PID_GAIN_MAX
#define PID_GAIN_MAX  1000.0f   // override per vehicle before including this header
#endif

static inline bool pp_pid_is_no_change(float g) {
    return fabsf(g - PID_NO_CHANGE) < 0.001f;
}

static inline bool pp_pid_gain_ok(float g) {
    if (pp_pid_is_no_change(g)) return true;
    if (g != g) return false;                                  // NaN
    return g >= -PID_GAIN_MAX && g <= PID_GAIN_MAX;            // also rejects +-Inf
}

static inline bool pp_pid_validate(const PidPayload *p) {
    for (int i = 0; i < 6; i++) {
        if (!pp_pid_gain_ok(p->gains_a[i]) || !pp_pid_gain_ok(p->gains_b[i]))
            return false;
    }
    return true;
}

// =============================================================================
// ATOMIC TWO-PAGE UPDATE
//
// A full retune spans two packets. Applying each page on arrival leaves the
// controller running new kp/ki against old kd/kff for at least one control
// tick -- indefinitely, if page 1 is lost.
//
// PidTransaction collects both pages of one txn_id before reporting
// completion. Feed every validated PidPayload to pp_pid_txn_apply_page() and
// copy kp/ki/kd/kff into the controller only on PP_PID_TXN_COMPLETE. A page
// carrying a different txn_id discards any half-finished transaction and
// starts a new one.
//
// To leave untouched gains at their live values, seed kp/ki/kd/kff from the
// controller after pp_pid_txn_init(); otherwise they default to 0.0f.
// =============================================================================
typedef enum {
    PP_PID_TXN_INCOMPLETE,     // page stored; waiting for its partner
    PP_PID_TXN_COMPLETE,       // both pages present; gain arrays ready to apply
    PP_PID_TXN_RESTARTED       // new txn_id arrived; previous partial discarded
} PpPidTxnResult;

typedef struct {
    uint8_t txn_id;
    bool    have_page0;
    bool    have_page1;
    float   kp[6], ki[6], kd[6], kff[6];
} PidTransaction;

static inline void pp_pid_txn_init(PidTransaction *txn) {
    memset(txn, 0, sizeof *txn);
}

static inline PpPidTxnResult pp_pid_txn_apply_page(PidTransaction *txn, const PidPayload *p) {
    PpPidTxnResult result = PP_PID_TXN_INCOMPLETE;

    if ((txn->have_page0 || txn->have_page1) && p->txn_id != txn->txn_id) {
        pp_pid_txn_init(txn);
        result = PP_PID_TXN_RESTARTED;
    }
    txn->txn_id = p->txn_id;

    float *a = NULL, *b = NULL;
    if (p->page == 0)      { a = txn->kp; b = txn->ki;  txn->have_page0 = true; }
    else if (p->page == 1) { a = txn->kd; b = txn->kff; txn->have_page1 = true; }
    // Unknown page numbers are ignored for forward compatibility.

    if (a) {
        for (int i = 0; i < 6; i++) {
            if (!pp_pid_is_no_change(p->gains_a[i])) a[i] = p->gains_a[i];
            if (!pp_pid_is_no_change(p->gains_b[i])) b[i] = p->gains_b[i];
        }
    }

    return (txn->have_page0 && txn->have_page1) ? PP_PID_TXN_COMPLETE : result;
}
