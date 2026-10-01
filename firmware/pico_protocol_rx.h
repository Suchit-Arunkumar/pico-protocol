// =============================================================================
// pico_protocol_rx.h
// Public API for the Pico-side receive ring buffer + resync-safe parser
// implemented in pico_protocol.c. See that file for the full explanation of
// the resync behaviour.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Reset the ring buffer to empty. Call once at startup.
void pp_rx_init(void);

// Feed up to `len` bytes into the ring buffer (e.g. from a UART RX ISR,
// DMA-drained region, or Serial.read() in a polling loop).
// Returns the number of bytes actually accepted — fewer than `len` if the
// buffer is full, so the caller can detect/report overflow instead of
// silently losing bytes.
size_t pp_rx_write(const uint8_t *data, size_t len);

// Bytes currently sitting in the buffer, waiting to be parsed.
size_t pp_rx_avail(void);

// Free space currently available for pp_rx_write().
size_t pp_rx_free(void);

// Count of bytes dropped by pp_rx_write() because the buffer was full at the
// time, accumulated since the last pp_rx_init(). Saturates at UINT16_MAX
// rather than wrapping, so a saturated reading is itself a signal ("at least
// this many, possibly more") instead of a misleadingly small wrapped count.
// A caller that never checks pp_rx_write()'s return value still has this to
// poll, so overflow is observable after the fact, not just detectable in the
// moment it happens.
uint16_t pp_rx_dropped_count(void);

// Try to pull exactly one valid packet out of the front of the buffer.
//
// On success: returns true, *out_type is set to one of TYPE_TELEMETRY /
// TYPE_CMD / TYPE_PID, and PAYLOAD_LEN (56) bytes are copied into
// out_payload (caller-owned buffer, must be at least PAYLOAD_LEN bytes —
// cast to TelemetryPayload*/CommandPayload*/PidPayload* based on *out_type).
//
// On failure: returns false. This means either not enough bytes have
// arrived yet for a full packet, or every candidate position examined so
// far failed a check (bad sync/length/type/CRC) and was dropped one byte at
// a time. Safe to call again after pp_rx_write() delivers more bytes.
bool pp_rx_try_parse(uint8_t *out_type, uint8_t *out_payload);
