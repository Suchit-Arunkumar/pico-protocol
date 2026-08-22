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
#include <string.h>   // memcpy, used by packetCRC() below

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
    uint8_t  reserved[5];      // bytes 51-55. MUST be zeroed by the sender --
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
_Static_assert(offsetof(TelemetryPayload, reserved)    == 51, "telemetry.reserved moved");

// TYPE 0x02 — CMD (Pi -> Pico)
typedef struct __attribute__((packed)) {
    float    current_x, current_y, current_z;
    float    current_roll, current_pitch, current_yaw;
    float    target_x, target_y, target_z;
    float    target_roll, target_pitch, target_yaw;
    uint8_t  armed;
    uint8_t  seq;
    uint8_t  reserved[6];      // bytes 50-55. MUST be zeroed by the sender --
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
_Static_assert(offsetof(CommandPayload, reserved)      == 50, "cmd.reserved moved");

// TYPE 0x03 — PID tuning (Pi -> Pico)
// page 0 -> gains_a = kp[6],  gains_b = ki[6]
// page 1 -> gains_a = kd[6],  gains_b = kff[6]
// Sentinel PID_NO_CHANGE (-999.0f) on any element means "leave unchanged".
typedef struct __attribute__((packed)) {
    uint8_t  page;
    uint8_t  reserved0[3];
    float    gains_a[6];
    float    gains_b[6];
    uint8_t  tail[4];          // bytes 52-55. Reserved for expansion; zero on send,
                               // ignore on receive. Covered by the CRC.
} PidPayload;
// --- Wire layout, TYPE 0x03 -------------------------------------------------
_Static_assert(HEADER_SIZE + sizeof(PidPayload) + CRC_SIZE == PACKET_SIZE,
               "PidPayload does not fill a 62-byte packet");
_Static_assert(offsetof(PidPayload, page)      ==  0, "pid.page moved");
_Static_assert(offsetof(PidPayload, reserved0) ==  1, "pid.reserved0 moved");
_Static_assert(offsetof(PidPayload, gains_a)   ==  4, "pid.gains_a moved");
_Static_assert(offsetof(PidPayload, gains_b)   == 28, "pid.gains_b moved");
_Static_assert(offsetof(PidPayload, tail)      == 52, "pid.tail moved");

#define PID_NO_CHANGE  -999.0f

// =============================================================================
// ZERO-INITIALISATION CONTRACT
//
// Every one of the 56 payload bytes is a named field -- there is no
// compiler-inserted padding in these packed structs. The trailing reserved[]
// / tail[] bytes are therefore real bytes that the sender chooses, and the
// CRC is computed over them.
//
// Senders MUST zero the whole struct before populating it:
//
//     TelemetryPayload t;
//     memset(&t, 0, sizeof(t));     // <-- not optional
//     t.depth_m = ...;
//
// Skipping this does NOT produce a detectable error. Stack garbage in the
// reserved bytes is covered by the CRC, so the packet is well-formed and the
// receiver accepts it. The costs are silent: uninitialised stack memory is
// transmitted on every frame, and two packets with identical logical content
// no longer produce identical bytes -- which makes golden-vector tests
// (compare built packet against known-good bytes) impossible to write.
//
// The firmware TX path lives outside this repo and is NOT verified here.
// =============================================================================

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
