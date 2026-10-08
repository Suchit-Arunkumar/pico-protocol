// =============================================================================
// fuzz_rx.c -- libFuzzer harness for the receive path.
//
// Each input is a byte stream fed to pp_rx_write() in chunks whose sizes come
// from the input itself, with pp_rx_try_parse() drained after every chunk, as
// a main loop would. Besides ASan/UBSan, it checks the parser's contract:
//
//   - every accepted packet has a known TYPE;
//   - while no bytes have been refused by a full buffer, re-framing an
//     accepted payload gives a packet whose bytes appear contiguously in the
//     input (nothing is accepted that was not sent). Once bytes are refused,
//     a spliced stream can pass a 16-bit CRC by chance, which is a property
//     of CRC-16, not a parser bug, so the check is skipped;
//   - the buffer never reports more bytes than it can hold.
//
//   make fuzz                       # builds build/fuzz_rx with clang
//   ./build/fuzz_rx -max_total_time=60
// =============================================================================

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "pico_protocol.h"
#include "pico_protocol_rx.h"

static int contains(const uint8_t *hay, size_t n, const uint8_t *needle, size_t m) {
    if (m > n) return 0;
    for (size_t i = 0; i + m <= n; i++)
        if (memcmp(&hay[i], needle, m) == 0) return 1;
    return 0;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    if (size < 1) return 0;

    // First byte seeds the chunk sizes; the rest is the stream.
    uint8_t seed = data[0];
    const uint8_t *stream = data + 1;
    size_t len = size - 1;

    pp_rx_init();

    size_t pos = 0;
    int refused = 0;
    uint8_t type, payload[PAYLOAD_LEN], frame[PACKET_SIZE];

    while (pos < len) {
        size_t chunk = 1u + (size_t)(seed % 64u);
        seed = (uint8_t)(seed * 33u + 7u);
        if (chunk > len - pos) chunk = len - pos;

        size_t accepted = pp_rx_write(&stream[pos], chunk);
        // Advance by what was offered: bytes refused by a full buffer are lost,
        // as they would be from a UART. The parser must still never invent a
        // packet from what it did receive.
        pos += chunk;
        if (accepted < chunk) refused = 1;

        if (pp_rx_avail() > 255u) abort();

        while (pp_rx_try_parse(&type, payload)) {
            if (type != TYPE_TELEMETRY && type != TYPE_CMD && type != TYPE_PID) abort();
            pp_frame(type, payload, frame);
            if (!refused && !contains(stream, len, frame, PACKET_SIZE)) abort();
        }
    }
    return 0;
}
