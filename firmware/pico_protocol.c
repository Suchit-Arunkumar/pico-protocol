// =============================================================================
// pico_protocol.c
// Firmware-side (Pico2/RP2350) RECEIVE-SIDE implementation for the Pi <-> Pico
// binary packet protocol defined in pico_protocol.h.
//
// This file is the piece that was previously missing from this repo: the
// header only defined packet structs, constants, and the CRC-16 math — it
// never actually consumed bytes off a wire. This file adds that: a ring
// buffer to hold incoming bytes, and a resync-safe parser that pulls valid
// packets out of it.
//
// Portable, plain C — no Arduino/Pico SDK calls. Feed it bytes from wherever
// your UART/USB-CDC RX happens (interrupt, DMA, or a polling loop like
// Serial.available()/Serial.read() on the arduino-pico core); it doesn't
// care where the bytes came from.
//
// RESYNC BEHAVIOUR (the whole point of this file)
// -------------------------------------------------------------------------
// The parser never assumes its current position in the buffer is a valid
// packet start. Every attempt walks through checks in order:
//
//   1. Are the next two bytes STX1 (0xAA) then STX2 (0x55)?
//   2. Does the LEN byte equal PAYLOAD_LEN (56), and is TYPE one of the
//      known packet types?
//   3. Does the CRC-16-CCITT over [LEN, TYPE, PAYLOAD] match the two CRC
//      bytes at the end of the packet?
//
// The moment ANY of those checks fails, the parser drops exactly one byte
// from the front of the buffer and starts over from check 1. It never
// pushes forward assuming alignment, and it never gets permanently stuck —
// a single dropped/corrupted byte costs at most one packet's width of
// resync time, not the rest of the session. This matters because 0xAA can
// legitimately appear inside real payload data by coincidence, so the
// header check alone isn't sufficient — the LEN/TYPE/CRC checks are what
// catch a false-positive header match.
// =============================================================================

#include <string.h>
#include <stdbool.h>
#include <assert.h>   // static_assert is a macro from here in C11; pico_protocol.h
                      // uses it but doesn't include this itself
#include "pico_protocol.h"
#include "pico_protocol_rx.h"

// =============================================================================
// RING BUFFER
//
// Safe circular buffer: writes check available space before inserting, so
// burst traffic can never silently overwrite unread data (pp_rx_write()
// returns how many bytes it actually accepted). The parser only ever reads
// via pp_rx_avail()/peek/eat below — nothing outside this file touches the
// head/tail indices directly.
// =============================================================================
#define RX_BUF_SIZE 256

static uint8_t g_rx[RX_BUF_SIZE];
static size_t  g_head = 0;   // next write position
static size_t  g_tail = 0;   // next read position

static inline size_t rx_avail(void) {
    return (g_head - g_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

// One slot is always left empty so a full buffer and an empty buffer never
// look identical (classic circular-buffer trick — avoids a separate count).
static inline size_t rx_free(void) {
    return (g_tail - g_head - 1 + RX_BUF_SIZE) % RX_BUF_SIZE;
}

static inline uint8_t rx_peek(size_t offset) {
    return g_rx[(g_tail + offset) % RX_BUF_SIZE];
}

static inline void rx_eat(size_t n) {
    g_tail = (g_tail + n) % RX_BUF_SIZE;
}

void pp_rx_init(void) {
    g_head = 0;
    g_tail = 0;
}

size_t pp_rx_free(void) {
    return rx_free();
}

size_t pp_rx_avail(void) {
    return rx_avail();
}

size_t pp_rx_write(const uint8_t *data, size_t len) {
    size_t n = 0;
    while (n < len && rx_free() > 0) {
        g_rx[g_head] = data[n];
        g_head = (g_head + 1) % RX_BUF_SIZE;
        n++;
    }
    return n;   // may be less than len if the buffer filled up
}

// =============================================================================
// RESYNC-SAFE PACKET PARSER
//
// Tries to extract exactly one valid packet from the front of the ring
// buffer. Returns true and fills out_type/out_payload on success. Returns
// false if there either aren't enough bytes yet, or every candidate
// position in the currently-available bytes failed validation (in which
// case all-but-fewer-than-one-packet's-worth of leading garbage has already
// been dropped, ready for the next call).
// =============================================================================
bool pp_rx_try_parse(uint8_t *out_type, uint8_t *out_payload) {
    while (rx_avail() >= PACKET_SIZE) {

        // --- Check 1: sync bytes -------------------------------------------
        if (rx_peek(0) != STX1 || rx_peek(1) != STX2) {
            rx_eat(1);
            continue;
        }

        // Pull the candidate packet out so we can validate/CRC it as a
        // contiguous buffer (cheap: PACKET_SIZE is 62 bytes).
        uint8_t pkt[PACKET_SIZE];
        for (size_t i = 0; i < PACKET_SIZE; i++) {
            pkt[i] = rx_peek(i);
        }

        uint8_t len  = pkt[2];
        uint8_t type = pkt[3];

        // --- Check 2: length + known type -----------------------------------
        bool known_type = (type == TYPE_TELEMETRY) ||
                           (type == TYPE_CMD)       ||
                           (type == TYPE_PID);

        if (len != PAYLOAD_LEN || !known_type) {
            rx_eat(1);
            continue;
        }

        // --- Check 3: CRC-16-CCITT over [LEN, TYPE, PAYLOAD] -----------------
        uint16_t calc = packetCRC(len, type, &pkt[4]);
        uint16_t rcvd = ((uint16_t)pkt[4 + PAYLOAD_LEN] << 8) | pkt[4 + PAYLOAD_LEN + 1];

        if (calc != rcvd) {
            rx_eat(1);
            continue;
        }

        // All three checks passed — this is a real packet.
        *out_type = type;
        memcpy(out_payload, &pkt[4], PAYLOAD_LEN);
        rx_eat(PACKET_SIZE);
        return true;
    }

    return false;
}
