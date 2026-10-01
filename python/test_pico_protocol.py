"""
Pi-side tests. Run from the repo root:

    python3 -m unittest discover -s python

The golden vectors below are duplicated verbatim in
firmware/test_pico_protocol.c. The C test builds each packet from the C
structs and compares against the same hex; this file builds it from the
Python encoder. Both passing is what establishes that the two
implementations produce byte-identical packets, rather than merely
agreeing on sizes and offsets.
"""

import unittest
import pico_protocol as p

GOLDEN_CMD = bytes.fromhex(
    'aa5538020000803f000000000000000000000000000000000000000000000000'
    '00000000000020c0000000000000000000000000012a0100000000006e8e'
)
GOLDEN_PID_PAGE0 = bytes.fromhex(
    'aa553803000109000000803f0000004000004040000080400000a0400000c040'
    '00c079c400c079c400c079c400c079c400c079c400c079c4000000006a60'
)

ZERO_POSE = dict(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)


class TestCrc(unittest.TestCase):
    def test_ibm3740_check_value(self):
        self.assertEqual(p.crc16_ccitt(b'123456789'), 0x29B1)


class TestGoldenVectors(unittest.TestCase):
    def test_cmd_matches_c(self):
        pkt = p.build_command_packet(dict(ZERO_POSE, x=1.0), dict(ZERO_POSE, z=-2.5),
                                     armed=1, seq=42)
        self.assertEqual(pkt, GOLDEN_CMD)

    def test_pid_page0_matches_c(self):
        pkts = p.build_pid_packets(kp=[1, 2, 3, 4, 5, 6],
                                   ki=[p.PID_NO_CHANGE] * 6, txn_id=9)
        self.assertEqual(pkts, [GOLDEN_PID_PAGE0])


class TestPidEncoder(unittest.TestCase):
    def test_full_retune_is_two_pages_sharing_txn_id(self):
        pkts = p.build_pid_packets(kp=[1] * 6, kd=[2] * 6, txn_id=5)
        self.assertEqual(len(pkts), 2)
        self.assertEqual([pk[4] for pk in pkts], [0, 1])          # page
        self.assertEqual([pk[4 + 2] for pk in pkts], [5, 5])      # txn_id
        self.assertEqual([pk[4 + 1] for pk in pkts], [p.PROTOCOL_VERSION] * 2)

    def test_page1_only_when_kd_or_kff_given(self):
        pkts = p.build_pid_packets(kp=[1] * 6)
        self.assertEqual(len(pkts), 1)
        self.assertEqual(pkts[0][4], 0)

    def test_wrong_length_rejected(self):
        with self.assertRaises(AssertionError):
            p.build_pid_packets(kp=[1, 2, 3])


class TestTelemetryParser(unittest.TestCase):
    def _telem(self, raw_depth=-1.0):
        import struct
        payload = struct.pack(p.TELEM_FMT, -1.0, raw_depth, *([0.0] * 6), *([1500] * 8),
                              1, 0, 1, p.PROTOCOL_VERSION, b'\x00' * 4)
        return p.frame(p.TYPE_TELEMETRY, payload)

    def test_resync_after_noise(self):
        buf = bytearray(b'\xaa\x55\x00\x13' + self._telem(-3.0))
        out = p.read_packet(buf)
        self.assertAlmostEqual(out['raw_depth_m'], -3.0, places=5)
        self.assertEqual(out['version'], p.PROTOCOL_VERSION)
        self.assertEqual(len(buf), 0)

    def test_corrupt_crc_rejected_next_packet_recovered(self):
        bad = bytearray(self._telem(-1.0))
        bad[-1] ^= 0xFF
        buf = bytearray(bad + self._telem(-2.0))
        out = p.read_packet(buf)
        self.assertAlmostEqual(out['raw_depth_m'], -2.0, places=5)

    def test_incomplete_returns_none(self):
        buf = bytearray(self._telem()[:40])
        self.assertIsNone(p.read_packet(buf))
        self.assertEqual(len(buf), 40)


if __name__ == '__main__':
    unittest.main()
