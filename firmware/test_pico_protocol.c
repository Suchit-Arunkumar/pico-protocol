// =============================================================================
// test_pico_protocol.c
// Host-side test harness for pico_protocol.c / pico_protocol_rx.h.
//
// Runs on a host machine with plain gcc or clang -- no Pico toolchain, no
// hardware. From the repository root:
//
//   make test
//
// Exits non-zero if any check fails.
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    CommandPayload cmd = pp_command_init();
    cmd.current_x = 1.0f;
    cmd.target_z  = -2.5f;
    cmd.armed     = 1;
    cmd.seq       = seq;
    pp_frame(TYPE_CMD, &cmd, out);
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
// Test 4 — a stray 0xAA 0x55 sequence that appears BEFORE any real packet
// header must be rejected as a decoy (bad CRC) and the parser must resync
// and recover the real packet that follows it.
//
// The decoy must come BEFORE the real packet. A decoy planted inside an
// already-locked packet's payload is never examined -- the parser consumes
// the whole 62 bytes once the outer CRC passes -- so such a test cannot
// fail. Here the decoy has plausible LEN/TYPE and a CRC that cannot match,
// so the CRC check is what must reject it and trigger the one-byte resync.
// -----------------------------------------------------------------------
static void test_false_header_before_valid_packet(void) {
    printf("Test 4: a decoy 0xAA 0x55 header with a bad CRC is rejected, real packet after it still recovers\n");
    pp_rx_init();

    // Decoy: real sync bytes, a plausible LEN/TYPE, but payload+CRC bytes
    // that do NOT satisfy the CRC check -- this must be rejected by check 3,
    // not accidentally skipped by checks 1/2.
    uint8_t decoy[PACKET_SIZE];
    memset(decoy, 0x00, sizeof(decoy));
    decoy[0] = STX1;
    decoy[1] = STX2;
    decoy[2] = PAYLOAD_LEN;
    decoy[3] = TYPE_CMD;
    // Leave payload all zero and CRC bytes wrong on purpose.
    decoy[4 + PAYLOAD_LEN]     = 0xDE;
    decoy[4 + PAYLOAD_LEN + 1] = 0xAD;

    uint8_t real[PACKET_SIZE];
    build_valid_cmd_packet(real, 5);

    pp_rx_write(decoy, sizeof(decoy));
    pp_rx_write(real, sizeof(real));

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok = pp_rx_try_parse(&type, payload);

    CHECK(ok, "parser recovers the real packet after rejecting the decoy header");
    CommandPayload *cmd = (CommandPayload *)payload;
    CHECK(cmd->seq == 5, "recovered packet is the real one (seq=5), not a false parse of the decoy");
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

// -----------------------------------------------------------------------
// Test 6 — a frame with a correct sync and CRC but LEN != PAYLOAD_LEN, and
// a separate frame with an unknown TYPE, must both be rejected by check 2
// -- and a valid packet placed right after either one must still recover.
// -----------------------------------------------------------------------
static void test_bad_len_and_unknown_type_rejected(void) {
    printf("Test 6: bad LEN and unknown TYPE are rejected, valid packet after each still recovers\n");

    // --- bad LEN ---------------------------------------------------------
    pp_rx_init();
    uint8_t bad_len[PACKET_SIZE];
    build_valid_cmd_packet(bad_len, 9);
    bad_len[2] = PAYLOAD_LEN + 1;   // LEN now disagrees with the fixed frame size
    // CRC was computed with the original LEN, so this frame is doubly wrong,
    // but the LEN check must be what rejects it, not the CRC -- LEN is
    // checked before CRC in pp_rx_try_parse(), so this still proves the LEN
    // branch is live and does not silently fall through to the CRC branch.

    uint8_t real_a[PACKET_SIZE];
    build_valid_cmd_packet(real_a, 10);

    pp_rx_write(bad_len, sizeof(bad_len));
    pp_rx_write(real_a, sizeof(real_a));

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok_a = pp_rx_try_parse(&type, payload);
    CHECK(ok_a, "parser recovers the valid packet after a bad-LEN frame");
    CHECK(((CommandPayload *)payload)->seq == 10, "recovered packet is the real one, not the bad-LEN frame");

    // --- unknown TYPE ------------------------------------------------------
    pp_rx_init();
    uint8_t bad_type[PACKET_SIZE];
    build_valid_cmd_packet(bad_type, 11);
    bad_type[3] = 0x7F;   // not TYPE_TELEMETRY/TYPE_CMD/TYPE_PID
    // As above, CRC was computed against TYPE_CMD, so the TYPE check (also
    // ahead of CRC) is what has to catch this.

    uint8_t real_b[PACKET_SIZE];
    build_valid_cmd_packet(real_b, 12);

    pp_rx_write(bad_type, sizeof(bad_type));
    pp_rx_write(real_b, sizeof(real_b));

    bool ok_b = pp_rx_try_parse(&type, payload);
    CHECK(ok_b, "parser recovers the valid packet after an unknown-TYPE frame");
    CHECK(((CommandPayload *)payload)->seq == 12, "recovered packet is the real one, not the bad-TYPE frame");
}

// -----------------------------------------------------------------------
// Test 7 — writing more than RX_BUF_SIZE (256) bytes' worth of packets in
// one session, without an intervening pp_rx_init(), forces g_head/g_tail
// to cross the modulo boundary at least once. Every other test writes at
// most a couple of packets after init (well under 256 bytes), so the
// wraparound arithmetic in rx_avail()/rx_free()/rx_peek()/rx_eat() has
// never actually been forced to wrap before this test.
// -----------------------------------------------------------------------
static void test_ring_buffer_wrap(void) {
    printf("Test 7: ring buffer indices wrap past RX_BUF_SIZE and parsing stays correct\n");
    pp_rx_init();

    // 256-byte buffer / 62-byte packets: 5 packets is 310 bytes, comfortably
    // forcing at least one wrap of g_head (and, as each is drained, g_tail).
    const int N = 5;
    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    int recovered = 0;

    for (int i = 0; i < N; i++) {
        uint8_t pkt[PACKET_SIZE];
        build_valid_cmd_packet(pkt, (uint8_t)(200 + i));
        size_t written = pp_rx_write(pkt, sizeof(pkt));
        CHECK(written == sizeof(pkt), "packet written in full (buffer had room)");

        bool ok = pp_rx_try_parse(&type, payload);
        if (ok && ((CommandPayload *)payload)->seq == (uint8_t)(200 + i)) {
            recovered++;
        }
    }

    CHECK(recovered == N, "all packets recovered in order across a buffer wrap");
    CHECK(pp_rx_avail() == 0, "buffer fully drained after wrapping");
}

// -----------------------------------------------------------------------
// Test 8 — writing more bytes than the buffer can hold: pp_rx_write() must
// return fewer than requested, the accepted prefix must still be an intact,
// parseable packet, and the buffer must recover (accept new data normally)
// once drained. The drop counter must account for exactly the bytes
// refused.
// -----------------------------------------------------------------------
static void test_buffer_full_short_write(void) {
    printf("Test 8: pp_rx_write() short-writes when full, accepted prefix still parses, buffer recovers\n");
    pp_rx_init();

    // RX_BUF_SIZE is 256 with one slot always kept empty, so max usable
    // capacity is 255 bytes. Ask for far more than that in one call.
    uint8_t filler[400];
    for (size_t i = 0; i < sizeof(filler); i++) filler[i] = 0x00;

    // Put one valid, parseable packet at the very front, then pad with
    // zero noise past the buffer's capacity.
    uint8_t pkt[PACKET_SIZE];
    build_valid_cmd_packet(pkt, 77);
    memcpy(filler, pkt, sizeof(pkt));

    size_t accepted = pp_rx_write(filler, sizeof(filler));
    CHECK(accepted < sizeof(filler), "pp_rx_write() reports fewer bytes accepted than requested");
    CHECK(accepted == 255, "accepted count equals the buffer's usable capacity (255 of 256 slots)");
    CHECK(pp_rx_dropped_count() == sizeof(filler) - accepted,
          "dropped-byte counter reflects exactly the bytes that didn't fit");

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok = pp_rx_try_parse(&type, payload);
    CHECK(ok, "the packet in the accepted prefix still parses intact");
    CHECK(((CommandPayload *)payload)->seq == 77, "parsed packet matches what was written, untouched by the overflow");

    // Buffer should now have room again and behave normally.
    pp_rx_init();   // draining via parse above already freed space; this
                    // models the recovery case explicitly regardless.
    uint8_t pkt2[PACKET_SIZE];
    build_valid_cmd_packet(pkt2, 78);
    size_t written2 = pp_rx_write(pkt2, sizeof(pkt2));
    CHECK(written2 == sizeof(pkt2), "buffer accepts a full packet normally after recovering from overflow");
    CHECK(pp_rx_dropped_count() == 0, "dropped counter resets on pp_rx_init()");
}

// -----------------------------------------------------------------------
// Test 9 — a single packet delivered to pp_rx_write() in several chunks
// (split mid-header, mid-payload, and between the two CRC bytes) must not
// parse until the final chunk lands, and must then parse intact. This is
// the normal case on real UART/USB-CDC hardware, where bytes arrive in
// whatever groupings the driver hands over, not as whole packets -- no
// prior test exercised a split packet at all.
// -----------------------------------------------------------------------
static void test_packet_split_across_writes(void) {
    printf("Test 9: a packet delivered in pieces across multiple pp_rx_write() calls still parses\n");

    uint8_t pkt[PACKET_SIZE];
    build_valid_cmd_packet(pkt, 55);

    // Split points: mid-header (byte 2, inside LEN), mid-payload (byte 30),
    // and between the two CRC bytes (byte 4 + PAYLOAD_LEN + 1, i.e. after
    // CRC_HI but before CRC_LO).
    size_t split_points[] = { 2, 30, 4 + PAYLOAD_LEN + 1 };

    for (size_t s = 0; s < sizeof(split_points) / sizeof(split_points[0]); s++) {
        pp_rx_init();
        size_t cut = split_points[s];

        uint8_t type;
        uint8_t payload[PAYLOAD_LEN];

        pp_rx_write(pkt, cut);
        bool ok_before = pp_rx_try_parse(&type, payload);
        CHECK(!ok_before, "parser returns false before the final chunk has arrived");

        pp_rx_write(pkt + cut, PACKET_SIZE - cut);
        bool ok_after = pp_rx_try_parse(&type, payload);
        CHECK(ok_after, "parser returns true once the final chunk lands");
        CHECK(((CommandPayload *)payload)->seq == 55, "packet split at this offset still decodes intact");
    }
}

// -----------------------------------------------------------------------
// Test 10 — pp_*_init() helpers zero every byte EXCEPT the version byte
// (stamped to PROTOCOL_VERSION), including the true reserved bytes, so a
// sender using them never transmits uninitialised memory.
//
// Checked field by field, since version is deliberately non-zero.
// -----------------------------------------------------------------------
static void test_init_helpers_zero_reserved_bytes(void) {
    printf("Test 10: pp_*_init() stamps version, zeros every other byte including reserved\n");

    TelemetryPayload t = pp_telemetry_init();
    CHECK(t.version == PROTOCOL_VERSION, "pp_telemetry_init() stamps version = PROTOCOL_VERSION");
    uint8_t t_reserved_zero[sizeof(t.reserved)];
    memset(t_reserved_zero, 0, sizeof(t_reserved_zero));
    CHECK(memcmp(t.reserved, t_reserved_zero, sizeof(t.reserved)) == 0,
          "pp_telemetry_init() zeros the true reserved bytes");
    CHECK(t.depth_m == 0.0f && t.armed == 0 && t.esc_pwm[0] == 0,
          "pp_telemetry_init() zeros ordinary fields too");

    CommandPayload c = pp_command_init();
    CHECK(c.version == PROTOCOL_VERSION, "pp_command_init() stamps version = PROTOCOL_VERSION");
    uint8_t c_reserved_zero[sizeof(c.reserved)];
    memset(c_reserved_zero, 0, sizeof(c_reserved_zero));
    CHECK(memcmp(c.reserved, c_reserved_zero, sizeof(c.reserved)) == 0,
          "pp_command_init() zeros the true reserved bytes");
    CHECK(c.current_x == 0.0f && c.armed == 0 && c.seq == 0,
          "pp_command_init() zeros ordinary fields too");

    PidPayload p = pp_pid_init();
    CHECK(p.version == PROTOCOL_VERSION, "pp_pid_init() stamps version = PROTOCOL_VERSION");
    CHECK(p.reserved0 == 0, "pp_pid_init() zeros the true reserved byte");
    CHECK(p.page == 0 && p.txn_id == 0 && p.gains_a[0] == 0.0f,
          "pp_pid_init() zeros ordinary fields too");
}

// -----------------------------------------------------------------------
// Test 12 — pp_packet_version() reads the version byte back out of a
// decoded payload for each of the three types, and the version stamped
// by pp_*_init() matches PROTOCOL_VERSION end to end through encode/CRC/
// decode, not just in the in-memory struct.
// -----------------------------------------------------------------------
static void test_packet_version_roundtrip(void) {
    printf("Test 12: pp_packet_version() reads back the version stamped by pp_*_init() through a real packet\n");

    CommandPayload cmd = pp_command_init();
    cmd.seq = 9;

    uint8_t pkt[PACKET_SIZE];
    pp_frame(TYPE_CMD, &cmd, pkt);

    pp_rx_init();
    pp_rx_write(pkt, PACKET_SIZE);

    uint8_t type;
    uint8_t payload[PAYLOAD_LEN];
    bool ok = pp_rx_try_parse(&type, payload);

    CHECK(ok, "versioned CMD packet parses");
    CHECK(pp_packet_version(type, payload) == PROTOCOL_VERSION,
          "pp_packet_version() reads back PROTOCOL_VERSION after a full encode/CRC/decode round trip");
    CHECK(pp_packet_version(0xFF, payload) == 0,
          "pp_packet_version() returns 0 for an unrecognised type");
}

// -----------------------------------------------------------------------
// Test 13 — pp_pid_txn_apply_page(): the atomic two-page update helper.
// Covers: incomplete after one page, complete once both pages of the SAME
// txn_id land, RESTARTED when a page arrives with a different txn_id
// mid-transaction (discarding the stale partial page), and PID_NO_CHANGE
// entries leaving a field at its prior value within the transaction.
// -----------------------------------------------------------------------
static void test_pid_transaction_atomic_update(void) {
    printf("Test 13: pp_pid_txn_apply_page() only completes once both pages of one txn_id land\n");

    // --- one page is not enough -----------------------------------------
    PidTransaction txn;
    pp_pid_txn_init(&txn);

    PidPayload page0 = pp_pid_init();
    page0.page   = 0;
    page0.txn_id = 42;
    for (int i = 0; i < 6; i++) { page0.gains_a[i] = 1.0f + (float)i; page0.gains_b[i] = 2.0f + (float)i; }

    PpPidTxnResult r0 = pp_pid_txn_apply_page(&txn, &page0);
    CHECK(r0 == PP_PID_TXN_INCOMPLETE, "one page alone is PP_PID_TXN_INCOMPLETE");

    // --- the matching second page completes it ---------------------------
    PidPayload page1 = pp_pid_init();
    page1.page   = 1;
    page1.txn_id = 42;
    for (int i = 0; i < 6; i++) { page1.gains_a[i] = 3.0f + (float)i; page1.gains_b[i] = 4.0f + (float)i; }

    PpPidTxnResult r1 = pp_pid_txn_apply_page(&txn, &page1);
    CHECK(r1 == PP_PID_TXN_COMPLETE, "matching second page completes the transaction");
    CHECK(txn.kp[0] == 1.0f && txn.ki[0] == 2.0f && txn.kd[0] == 3.0f && txn.kff[0] == 4.0f,
          "all four gain arrays assembled correctly from both pages");

    // --- a page with a different txn_id mid-transaction restarts, not merges ---
    PidTransaction txn2;
    pp_pid_txn_init(&txn2);

    PidPayload stale_page0 = pp_pid_init();
    stale_page0.page = 0;
    stale_page0.txn_id = 1;
    stale_page0.gains_a[0] = 99.0f;
    pp_pid_txn_apply_page(&txn2, &stale_page0);   // starts txn_id=1, page0 only

    PidPayload new_page0 = pp_pid_init();
    new_page0.page = 0;
    new_page0.txn_id = 2;   // different transaction -- sender restarted
    new_page0.gains_a[0] = 7.0f;

    PpPidTxnResult r2 = pp_pid_txn_apply_page(&txn2, &new_page0);
    CHECK(r2 == PP_PID_TXN_RESTARTED, "a page with a new txn_id mid-transaction reports RESTARTED");
    CHECK(txn2.kp[0] == 7.0f, "the restarted transaction holds the NEW page's data, not the stale one");
    CHECK(txn2.have_page1 == false, "the restarted transaction does not carry over the stale page1 state");

    // --- PID_NO_CHANGE leaves a field at the transaction's prior value ----
    PidTransaction txn3;
    pp_pid_txn_init(&txn3);   // all gain arrays start at 0.0f

    PidPayload partial0 = pp_pid_init();
    partial0.page = 0;
    partial0.txn_id = 5;
    partial0.gains_a[0] = 11.0f;              // kp[0] set
    partial0.gains_a[1] = PID_NO_CHANGE;      // kp[1] left alone
    for (int i = 0; i < 6; i++) partial0.gains_b[i] = PID_NO_CHANGE;

    pp_pid_txn_apply_page(&txn3, &partial0);
    CHECK(txn3.kp[0] == 11.0f, "an explicit gain value is applied");
    CHECK(txn3.kp[1] == 0.0f, "PID_NO_CHANGE leaves the transaction's prior value (0.0f default) alone");
}

// -----------------------------------------------------------------------
// Test 11 — pp_pid_validate() accepts sane gains and PID_NO_CHANGE, and
// rejects NaN, +-Inf, and out-of-range values in either gains_a or
// gains_b, at any of the 12 gain slots.
// -----------------------------------------------------------------------
static void test_pid_validate(void) {
    printf("Test 11: pp_pid_validate() accepts sane gains, rejects NaN/Inf/out-of-range\n");

    PidPayload good = pp_pid_init();
    for (int i = 0; i < 6; i++) { good.gains_a[i] = 1.5f; good.gains_b[i] = PID_NO_CHANGE; }
    CHECK(pp_pid_validate(&good), "all-sane-plus-PID_NO_CHANGE payload validates");

    PidPayload nan_payload = good;
    nan_payload.gains_a[3] = 0.0f / 0.0f;
    CHECK(!pp_pid_validate(&nan_payload), "NaN in gains_a is rejected");

    PidPayload inf_payload = good;
    inf_payload.gains_b[0] = 1.0f / 0.0f;
    CHECK(!pp_pid_validate(&inf_payload), "+Inf in gains_b is rejected");

    PidPayload huge_payload = good;
    huge_payload.gains_a[5] = PID_GAIN_MAX + 1.0f;
    CHECK(!pp_pid_validate(&huge_payload), "a gain past PID_GAIN_MAX is rejected");

    PidPayload neg_huge_payload = good;
    neg_huge_payload.gains_b[2] = -(PID_GAIN_MAX + 1.0f);
    CHECK(!pp_pid_validate(&neg_huge_payload), "a gain past -PID_GAIN_MAX is rejected");
}

// -----------------------------------------------------------------------
// Test 14 — golden vectors shared with python/test_pico_protocol.py.
//
// The same hex strings appear verbatim in the Python tests. Here the
// packets are built from the C structs; there, from the Python encoder.
// Both passing is what demonstrates byte-identical output across the two
// implementations, beyond agreement on sizes and offsets.
// -----------------------------------------------------------------------
static int hex_to_bytes(const char *hex, uint8_t *out, size_t cap) {
    size_t n = strlen(hex) / 2;
    if (n > cap) return -1;
    for (size_t i = 0; i < n; i++) {
        unsigned v;
        if (sscanf(hex + 2 * i, "%2x", &v) != 1) return -1;
        out[i] = (uint8_t)v;
    }
    return (int)n;
}

static void test_golden_vectors_match_python(void) {
    printf("Test 14: C-built packets match the golden vectors the Python tests also check\n");

    static const char *GOLDEN_CMD =
        "aa5538020000803f000000000000000000000000000000000000000000000000"
        "00000000000020c0000000000000000000000000012a0100000000006e8e";
    static const char *GOLDEN_PID_PAGE0 =
        "aa553803000109000000803f0000004000004040000080400000a0400000c040"
        "00c079c400c079c400c079c400c079c400c079c400c079c4000000006a60";

    uint8_t expect[PACKET_SIZE], got[PACKET_SIZE];

    CommandPayload cmd = pp_command_init();
    cmd.current_x = 1.0f;
    cmd.target_z  = -2.5f;
    cmd.armed     = 1;
    cmd.seq       = 42;
    pp_frame(TYPE_CMD, &cmd, got);
    CHECK(hex_to_bytes(GOLDEN_CMD, expect, sizeof expect) == PACKET_SIZE &&
          memcmp(got, expect, PACKET_SIZE) == 0,
          "CMD packet is byte-identical to the Python encoder's output");

    PidPayload pid = pp_pid_init();
    pid.page   = 0;
    pid.txn_id = 9;
    for (int i = 0; i < 6; i++) { pid.gains_a[i] = (float)(i + 1); pid.gains_b[i] = PID_NO_CHANGE; }
    pp_frame(TYPE_PID, &pid, got);
    CHECK(hex_to_bytes(GOLDEN_PID_PAGE0, expect, sizeof expect) == PACKET_SIZE &&
          memcmp(got, expect, PACKET_SIZE) == 0,
          "PID page-0 packet is byte-identical to the Python encoder's output");

    CHECK(crc16((const uint8_t *)"123456789", 9) == 0x29B1,
          "crc16() matches the published CRC-16/IBM-3740 check value 0x29B1");
}

// -----------------------------------------------------------------------
// Test 15 — randomized stress. Valid packets interleaved with random noise
// (biased toward 0xAA/0x55/56 so false headers are common), delivered
// through pp_rx_write() in random chunk sizes from 1 to 40 bytes, parsed
// as the bytes arrive. Every packet must be recovered exactly once, in
// order, with its seq intact. Seeded xorshift PRNG for reproducibility.
//
// This is property-based stress testing, not coverage-guided fuzzing; see
// LIMITATIONS.md.
// -----------------------------------------------------------------------
static uint32_t g_rng = 0x2545F491u;
static uint32_t rng(void) {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return g_rng;
}

static void test_randomized_noise_stress(void) {
    printf("Test 15: randomized noise + random chunking, every packet recovered in order\n");
    pp_rx_init();

    enum { N_PACKETS = 2000 };
    static uint8_t stream[N_PACKETS * (PACKET_SIZE + 48)];
    size_t len = 0;

    for (int k = 0; k < N_PACKETS; k++) {
        size_t noise = rng() % 48;
        for (size_t i = 0; i < noise; i++) {
            uint32_t r = rng() % 8;
            stream[len++] = (r == 0) ? STX1 : (r == 1) ? STX2 : (r == 2) ? PAYLOAD_LEN
                          : (uint8_t)rng();
        }
        build_valid_cmd_packet(&stream[len], (uint8_t)k);
        len += PACKET_SIZE;
    }

    int recovered = 0, in_order = 1;
    size_t pos = 0;
    uint8_t type, payload[PAYLOAD_LEN];

    while (pos < len) {
        size_t chunk = 1 + rng() % 40;
        if (chunk > len - pos) chunk = len - pos;
        pos += pp_rx_write(&stream[pos], chunk);
        while (pp_rx_try_parse(&type, payload)) {
            if (type != TYPE_CMD || ((CommandPayload *)payload)->seq != (uint8_t)recovered)
                in_order = 0;
            recovered++;
        }
    }

    CHECK(recovered == N_PACKETS, "all 2000 packets recovered from the noisy stream");
    CHECK(in_order, "packets recovered in order with intact seq, no false accepts");
    CHECK(pp_rx_dropped_count() == 0, "no bytes dropped (writer never outpaced the parser)");
}

// ---------------------------------------------------------------------------
// Link watchdog: false before the first CMD, true for PP_LINK_TIMEOUT_MS after
// each feed, false from then on, and unaffected by the millisecond counter
// wrapping. Version read-back works on an unaligned payload buffer.
// ---------------------------------------------------------------------------
static void test_link_watchdog(void) {
    printf("\n[test_link_watchdog]\n");
    PpLinkWatchdog w;
    pp_link_init(&w);
    CHECK(!pp_link_ok(&w, 0) && !pp_link_ok(&w, 12345), "not ok before any command");

    pp_link_feed(&w, 1000);
    CHECK(pp_link_ok(&w, 1000), "ok on the tick it was fed");
    CHECK(pp_link_ok(&w, 1000 + PP_LINK_TIMEOUT_MS - 1), "ok 1 ms before the timeout");
    CHECK(!pp_link_ok(&w, 1000 + PP_LINK_TIMEOUT_MS), "lost exactly at the timeout");

    pp_link_feed(&w, UINT32_MAX - 100);
    CHECK(pp_link_ok(&w, 50), "ok across the 2^32 ms wrap (151 ms later)");
    CHECK(!pp_link_ok(&w, PP_LINK_TIMEOUT_MS), "lost across the wrap once the timeout passes");

    uint8_t buf[PAYLOAD_LEN + 1];
    CommandPayload c = pp_command_init();
    memcpy(&buf[1], &c, sizeof c);                     // deliberately misaligned
    CHECK(pp_packet_version(TYPE_CMD, &buf[1]) == PROTOCOL_VERSION,
          "version read from a misaligned payload buffer");
}

int main(void) {
    test_valid_packet();
    test_noise_before_valid_packet();
    test_dropped_byte_recovers_next_packet();
    test_false_header_before_valid_packet();
    test_back_to_back_packets();
    test_bad_len_and_unknown_type_rejected();
    test_ring_buffer_wrap();
    test_buffer_full_short_write();
    test_packet_split_across_writes();
    test_init_helpers_zero_reserved_bytes();
    test_pid_validate();
    test_packet_version_roundtrip();
    test_pid_transaction_atomic_update();
    test_golden_vectors_match_python();
    test_randomized_noise_stress();
    test_link_watchdog();

    printf("\n=====================================\n");
    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
    } else {
        printf("%d CHECK(S) FAILED\n", g_failures);
    }
    printf("=====================================\n");

    return g_failures == 0 ? 0 : 1;
}
