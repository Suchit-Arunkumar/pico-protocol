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

// =============================================================================
// FRAMING
// =============================================================================
#define STX1         0xAA
#define STX2         0x55
#define PAYLOAD_LEN  56
#define PACKET_SIZE  (4 + PAYLOAD_LEN + 2)   // 62 bytes total

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
    uint8_t  reserved[5];      // zero-padding
} TelemetryPayload;
static_assert(sizeof(TelemetryPayload) == PAYLOAD_LEN, "TelemetryPayload size mismatch");

// TYPE 0x02 — CMD (Pi -> Pico)
typedef struct __attribute__((packed)) {
    float    current_x, current_y, current_z;
    float    current_roll, current_pitch, current_yaw;
    float    target_x, target_y, target_z;
    float    target_roll, target_pitch, target_yaw;
    uint8_t  armed;
    uint8_t  seq;
    uint8_t  reserved[6];
} CommandPayload;
static_assert(sizeof(CommandPayload) == PAYLOAD_LEN, "CommandPayload size mismatch");

// TYPE 0x03 — PID tuning (Pi -> Pico)
// page 0 -> gains_a = kp[6],  gains_b = ki[6]
// page 1 -> gains_a = kd[6],  gains_b = kff[6]
// Sentinel PID_NO_CHANGE (-999.0f) on any element means "leave unchanged".
typedef struct __attribute__((packed)) {
    uint8_t  page;
    uint8_t  reserved0[3];
    float    gains_a[6];
    float    gains_b[6];
    uint8_t  tail[4];
} PidPayload;
static_assert(sizeof(PidPayload) == PAYLOAD_LEN, "PidPayload size mismatch");

#define PID_NO_CHANGE  -999.0f

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

static inline uint16_t packetCRC(uint8_t len, uint8_t type, const uint8_t *payload) {
    uint8_t ci[2 + PAYLOAD_LEN];
    ci[0] = len; ci[1] = type;
    memcpy(&ci[2], payload, PAYLOAD_LEN);
    return crc16(ci, sizeof(ci));
}
