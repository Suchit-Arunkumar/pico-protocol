"""
Hardware-free round trip through pico_protocol.py.

Builds a CMD packet and a PID update, then hand-assembles a TELEMETRY packet
(as the Pico would send it), prefixes it with junk bytes, and decodes it
through read_packet() to exercise the resync path.
"""

import struct
from pico_protocol import (
    PROTOCOL_VERSION, TYPE_TELEMETRY, TELEM_FMT,
    frame, build_command_packet, build_pid_packets, read_packet,
)

# --- CMD (Pi -> Pico) ---------------------------------------------------------
pose   = dict(x=0.0, y=0.0, z=-1.2, roll=0.0, pitch=0.0, yaw=45.0)
target = dict(x=1.0, y=0.0, z=-1.5, roll=0.0, pitch=0.0, yaw=45.0)
cmd_pkt = build_command_packet(pose, target, armed=1)
print(f"CMD packet ({len(cmd_pkt)} bytes): {cmd_pkt.hex()}")

# --- PID (Pi -> Pico): full retune, both pages share one txn_id ---------------
pid_pkts = build_pid_packets(kp=[0, 0, 40, 2, 2, 0], ki=[0, 0, 1, 0, 0, 0],
                             kd=[0, 0, 5, 0.5, 0.5, 0], kff=[0] * 6, txn_id=7)
print(f"PID update: {len(pid_pkts)} packets, txn_id=7")

# --- TELEMETRY (Pico -> Pi), with leading junk to force a resync --------------
payload = struct.pack(
    TELEM_FMT,
    -1.2, -1.18,                       # depth_m (echo), raw_depth_m
    0.1, 0.0, -0.4, 0.0, 0.0, 0.05,    # pid_u[6]
    *([1500] * 8),                     # esc_pwm[8]
    1, 0, 1,                           # armed, sat_flags, link_ok
    PROTOCOL_VERSION,
    b'\x00' * 4,
)
buf = bytearray(b'\x00\xff\xaa\x12' + frame(TYPE_TELEMETRY, payload))
decoded = read_packet(buf)

print("\nDecoded TELEMETRY:")
for k, v in decoded.items():
    print(f"  {k}: {v}")

assert abs(decoded['raw_depth_m'] - (-1.18)) < 1e-4
assert decoded['armed'] == 1
assert decoded['esc_pwm'] == [1500] * 8
assert decoded['version'] == PROTOCOL_VERSION
assert len(buf) == 0
print("\nRound trip OK.")
