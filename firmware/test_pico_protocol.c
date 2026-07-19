// =============================================================================
// test_pico_protocol.c
// Host-side test harness for pico_protocol.c / pico_protocol_rx.h.
//
// Compiles and runs on a laptop (plain gcc, no Pico/Arduino toolchain, no
// hardware) so the resync logic can be proven correct BEFORE it ever touches
// a real UART. Build & run:
//
//   gcc -Wall -Wextra -o test_pico_protocol test_pico_protocol.c pico_protocol.c
//   ./test_pico_protocol
//
// Every test ends with a pass/fail printout; the process exits non-zero if
// any test fails, so it can be dropped straight into a CI step later.
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "pico_protocol.h"
#include "pico_protocol_rx.h"

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  [PASS] %s\n", msg); } \
    else      { printf("  [FAIL] %s\n", msg); g_failures++; } \
} while (0)

// Build a full, valid TYPE_CMD packet into `out` (must be PACKET_SIZE bytes).
// Uses a distinctive, checkable seq byte so tests can confirm WHICH packet
// was recovered after corruption/noise.
static void build_valid_cmd_packet(uint8_t *out, uint8_t seq) {
    CommandPayload cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.current_x = 1.0f;
    cmd.target_z  = -2.5f;
    cmd.armed     = 1;
    cmd.seq       = seq;

    out[0] = STX1;
    out[1] = STX2;
    out[2] = PAYLOAD_LEN;
    out[3] = TYPE_CMD;
    memcpy(&out[4], &cmd, PAYLOAD_LEN);

    uint16_t crc = packetCRC(PAYLOAD_LEN, TYPE_CMD, &out[4]);
    out[4 + PAYLOAD_LEN]     = (crc >> 8) & 0xFF;
    out[4 + PAYLOAD_LEN + 1] = crc & 0xFF;
}

// -----------------------------------------------------------------------
// Test 1 — a single clean packet parses correctly.
// -----------------------------------------------------------------------
static void test_valid_packet(void) {
    printf("Test 1: valid packet parses cleanly\n");
    pp_rx_init();

    uint8_t pkt[PACKET_SIZE];
    build_valid_cmd_packet(pkt, 42);
    pp_rx_write(pkt, PACKET_SIZE);

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok = pp_rx_try_parse(&type, payload);

    CHECK(ok, "parser returns true for a valid packet");
    CHECK(type == TYPE_CMD, "type decoded as TYPE_CMD");

    CommandPayload *cmd = (CommandPayload *)payload;
    CHECK(cmd->seq == 42, "seq field round-trips correctly");
    CHECK(pp_rx_avail() == 0, "buffer fully drained after one packet");
}

// -----------------------------------------------------------------------
// Test 2 — random noise before a valid packet is skipped, not fatal.
// -----------------------------------------------------------------------
static void test_noise_before_valid_packet(void) {
    printf("Test 2: junk bytes before a valid packet are skipped\n");
    pp_rx_init();

    uint8_t noise[10] = {0x00, 0xFF, 0x12, 0xAA, 0x00, 0x55, 0xAA, 0x99, 0x11, 0xAA};
    pp_rx_write(noise, sizeof(noise));

    uint8_t pkt[PACKET_SIZE];
    build_valid_cmd_packet(pkt, 7);
    pp_rx_write(pkt, PACKET_SIZE);

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok = pp_rx_try_parse(&type, payload);

    CHECK(ok, "parser recovers the valid packet after leading noise");
    CommandPayload *cmd = (CommandPayload *)payload;
    CHECK(cmd->seq == 7, "correct packet (seq=7) recovered, not a false match in the noise");
}

// -----------------------------------------------------------------------
// Test 3 — a byte deleted mid-packet corrupts packet A, but the parser
// resyncs in time to correctly recover packet B right after it.
// This is the core "dropped byte on the wire" scenario.
// -----------------------------------------------------------------------
static void test_dropped_byte_recovers_next_packet(void) {
    printf("Test 3: a byte dropped mid-packet doesn't take down the next packet\n");
    pp_rx_init();

    uint8_t pkt_a[PACKET_SIZE];
    uint8_t pkt_b[PACKET_SIZE];
    build_valid_cmd_packet(pkt_a, 11);
    build_valid_cmd_packet(pkt_b, 22);

    // Simulate one byte vanishing from the middle of packet A's payload.
    uint8_t corrupted_a[PACKET_SIZE - 1];
    size_t drop_index = 20; // somewhere in the payload
    memcpy(corrupted_a, pkt_a, drop_index);
    memcpy(corrupted_a + drop_index, pkt_a + drop_index + 1, PACKET_SIZE - drop_index - 1);

    pp_rx_write(corrupted_a, sizeof(corrupted_a));
    pp_rx_write(pkt_b, PACKET_SIZE);

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];

    // First parse attempt should fail to produce packet A (its CRC won't
    // match once shifted) but must not get stuck — it should keep hunting
    // and land on packet B.
    bool ok = pp_rx_try_parse(&type, payload);
    CHECK(ok, "parser still finds a valid packet after the corrupted one");

    CommandPayload *cmd = (CommandPayload *)payload;
    CHECK(cmd->seq == 22, "recovered packet is B (seq=22), A was correctly discarded");
}

// -----------------------------------------------------------------------
// Test 4 — a stray 0xAA 0x55 sequence embedded inside real payload data
// must NOT be mistaken for a packet header.
// -----------------------------------------------------------------------
static void test_false_header_inside_payload(void) {
    printf("Test 4: coincidental 0xAA 0x55 inside payload doesn't cause a false lock\n");
    pp_rx_init();

    uint8_t pkt[PACKET_SIZE];
    build_valid_cmd_packet(pkt, 5);

    // Force a 0xAA 0x55 pair into the middle of the payload on purpose.
    pkt[10] = STX1;
    pkt[11] = STX2;
    // Recompute CRC since we just mutated the payload.
    uint16_t crc = packetCRC(PAYLOAD_LEN, TYPE_CMD, &pkt[4]);
    pkt[4 + PAYLOAD_LEN]     = (crc >> 8) & 0xFF;
    pkt[4 + PAYLOAD_LEN + 1] = crc & 0xFF;

    pp_rx_write(pkt, PACKET_SIZE);

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok = pp_rx_try_parse(&type, payload);

    CHECK(ok, "the real (outer) packet is still parsed correctly");
    CommandPayload *cmd = (CommandPayload *)payload;
    CHECK(cmd->seq == 5, "seq matches the real packet, not a false parse starting at the embedded header");
}

// -----------------------------------------------------------------------
// Test 5 — two valid packets back-to-back with zero gap both come out,
// in order, from repeated calls.
// -----------------------------------------------------------------------
static void test_back_to_back_packets(void) {
    printf("Test 5: two packets spliced with no gap both parse in order\n");
    pp_rx_init();

    uint8_t pkt_a[PACKET_SIZE], pkt_b[PACKET_SIZE];
    build_valid_cmd_packet(pkt_a, 100);
    build_valid_cmd_packet(pkt_b, 101);

    pp_rx_write(pkt_a, PACKET_SIZE);
    pp_rx_write(pkt_b, PACKET_SIZE);

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];

    bool ok1 = pp_rx_try_parse(&type, payload);
    uint8_t seq1 = ((CommandPayload *)payload)->seq;

    bool ok2 = pp_rx_try_parse(&type, payload);
    uint8_t seq2 = ((CommandPayload *)payload)->seq;

    CHECK(ok1 && seq1 == 100, "first packet (seq=100) recovered first");
    CHECK(ok2 && seq2 == 101, "second packet (seq=101) recovered right after, no gap needed");
}

int main(void) {
    test_valid_packet();
    test_noise_before_valid_packet();
    test_dropped_byte_recovers_next_packet();
    test_false_header_inside_payload();
    test_back_to_back_packets();

    printf("\n=====================================\n");
    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
    } else {
        printf("%d CHECK(S) FAILED\n", g_failures);
    }
    printf("=====================================\n");

    return g_failures == 0 ? 0 : 1;
}
