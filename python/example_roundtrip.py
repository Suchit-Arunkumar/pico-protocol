"""
Round-trip sanity check for pico_protocol.py.

Builds a CMD packet, then manually assembles a matching TELEMETRY packet
(as if it came back from the Pico) and decodes it through read_packet().
No hardware required -- this just proves the encode/decode/CRC/framing
logic is internally consistent.
"""

import struct
from pico_protocol import (
    STX1, STX2, PAYLOAD_LEN, TYPE_TELEMETRY, TELEM_FMT,
    packet_crc, build_command_packet, read_packet,
)

# --- 1. Build a CMD packet -----------------------------------------------
pose   = dict(x=0.0, y=0.0, z=-1.2, roll=0.0, pitch=0.0, yaw=45.0)
target = dict(x=1.0, y=0.0, z=-1.5, roll=0.0, pitch=0.0, yaw=45.0)
cmd_pkt = build_command_packet(pose, target, armed=1)
print(f"CMD packet: {len(cmd_pkt)} bytes -> {cmd_pkt.hex()}")

# --- 2. Hand-build a TELEMETRY packet, as the Pico would send -----------
payload = struct.pack(
    TELEM_FMT,
    -1.2, -1.18,                    # depth_m (echo), raw_depth_m
    0.1, 0.0, -0.4, 0.0, 0.0, 0.05, # pid_u[6]
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,         # esc_pwm[8]
    1, 0, 1,                        # armed, sat_flags, link_ok
    b'\x00' * 5,                    # reserved
)
crc = packet_crc(PAYLOAD_LEN, TYPE_TELEMETRY, payload)
telem_pkt = bytes([STX1, STX2, PAYLOAD_LEN, TYPE_TELEMETRY]) + payload + bytes([crc >> 8, crc & 0xFF])

# --- 3. Feed it through read_packet(), with some junk bytes in front to
#        prove the resync logic works too ---------------------------------
buf = bytearray(b'\x00\xff\x12' + telem_pkt)
decoded = read_packet(buf)

print(f"\nDecoded TELEMETRY packet:")
for k, v in decoded.items():
    print(f"  {k}: {v}")

assert abs(decoded['raw_depth_m'] - (-1.18)) < 1e-4
assert decoded['armed'] == 1
assert decoded['esc_pwm'] == [1500] * 8
print("\nRound trip OK.")
