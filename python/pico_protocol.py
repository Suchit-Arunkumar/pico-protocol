"""
pico_protocol.py
=================
Binary packet protocol for the Pi <-> Pico2 serial link.

Wire format (little-endian):

    [ STX1 ][ STX2 ][ LEN ][ TYPE ][ ... PAYLOAD (LEN bytes) ... ][ CRC_HI ][ CRC_LO ]
      0xAA    0x55     1B     1B              56 bytes                  1B       1B

  - STX1/STX2  : fixed 2-byte sync sequence (0xAA, 0x55)
  - LEN        : payload length in bytes (56 for all packet types below)
  - TYPE       : packet type (see PacketType)
  - PAYLOAD    : type-specific struct, always 56 bytes, little-endian
  - CRC        : CRC-16-CCITT (poly 0x1021, init 0xFFFF) over [LEN, TYPE, PAYLOAD...]

  Total packet size = 4 + LEN + 2 = 62 bytes.

Packet types implemented on this (Pi) side:
    0x01  TELEMETRY   Pico -> Pi   56-byte status snapshot
    0x02  CMD         Pi -> Pico   fused pose + target + arm state

A third type, 0x03 (PID live-tuning), is defined on the firmware side
(see firmware/pico_protocol.h) but does not yet have a Python-side
encoder here.

This module has no side effects and no I/O — it only builds and parses
byte buffers. Wiring it to an actual serial port is left to the caller.
"""

import struct

# =============================================================================
# PROTOCOL CONSTANTS
# =============================================================================

STX1 = 0xAA
STX2 = 0x55

PAYLOAD_LEN = 56
PACKET_SIZE = 4 + PAYLOAD_LEN + 2   # STX1 STX2 LEN TYPE + payload + CRC_HI CRC_LO

TYPE_TELEMETRY = 0x01   # Pico -> Pi
TYPE_CMD       = 0x02   # Pi   -> Pico
TYPE_PID       = 0x03   # Pi   -> Pico (firmware-side only, see note above)

# TELEMETRY payload (56 bytes): depth_m, raw_depth_m, pid_u[6], esc_pwm[8],
# armed, sat_flags, link_ok, reserved[5]
TELEM_FMT = '<ff6f8H3B5s'

# CMD payload (56 bytes): current pose (6f) + target pose (6f) + armed + seq
# + reserved[6]
CMD_FMT = '<12f2B6s'


# =============================================================================
# CRC-16-CCITT
# =============================================================================

def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT, polynomial 0x1021, initial value 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def packet_crc(length: int, ptype: int, payload: bytes) -> int:
    """CRC is computed over [LEN, TYPE, PAYLOAD] -- NOT over STX1/STX2."""
    return crc16_ccitt(bytes([length, ptype]) + payload)


# =============================================================================
# ENCODE — build a CMD packet (Pi -> Pico)
# =============================================================================

_seq = 0


def build_command_packet(pose: dict, target: dict, armed: int = 0) -> bytes:
    """
    Build a TYPE_CMD packet from the current fused pose and a target pose.

    pose / target dicts must each have keys: x, y, z, roll, pitch, yaw.
    Sequence number auto-increments (mod 256) on every call.
    """
    global _seq
    _seq = (_seq + 1) & 0xFF

    payload = struct.pack(
        CMD_FMT,
        pose['x'], pose['y'], pose['z'],
        pose['roll'], pose['pitch'], pose['yaw'],
        target['x'], target['y'], target['z'],
        target['roll'], target['pitch'], target['yaw'],
        armed & 0xFF, _seq,
        b'\x00' * 6,
    )
    crc = packet_crc(PAYLOAD_LEN, TYPE_CMD, payload)
    return (
        bytes([STX1, STX2, PAYLOAD_LEN, TYPE_CMD])
        + payload
        + bytes([crc >> 8, crc & 0xFF])
    )


# =============================================================================
# DECODE — parse a TELEMETRY packet (Pico -> Pi)
# =============================================================================

def decode_telemetry(payload: bytes) -> dict:
    """Decode a 56-byte, already CRC-verified TELEMETRY payload."""
    (depth_m, raw_depth_m,
     u0, u1, u2, u3, u4, u5,
     pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7,
     armed, sat_flags, link_ok,
     _reserved) = struct.unpack(TELEM_FMT, payload)

    return {
        'depth_m': depth_m,          # Pi's own current_z, echoed back -- not
                                      # a fresh reading, do not feed back into
                                      # your own estimator
        'raw_depth_m': raw_depth_m,  # actual pressure-sensor reading
        'pid_u': [u0, u1, u2, u3, u4, u5],
        'esc_pwm': [pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7],
        'armed': armed,
        'sat_flags': sat_flags,      # bit0=vert, bit1=horiz, bit2=yaw saturation
        'link_ok': link_ok,
    }


def read_packet(buf: bytearray):
    """
    Try to extract one valid TELEMETRY packet from the front of `buf`.

    Returns a decoded dict on success, or None if `buf` doesn't yet contain
    a full valid packet. On a bad sync byte, wrong length/type, or CRC
    mismatch, resyncs by dropping a single byte and retrying -- this mirrors
    the framing/resync behaviour on the firmware side, so a corrupted or
    misaligned stream self-recovers within one packet's worth of bytes
    instead of stalling.

    `buf` is mutated in place: consumed/discarded bytes are removed.
    """
    while len(buf) >= PACKET_SIZE:
        if buf[0] != STX1 or buf[1] != STX2:
            del buf[0]
            continue

        pkt = bytes(buf[:PACKET_SIZE])
        length, ptype = pkt[2], pkt[3]

        if length != PAYLOAD_LEN or ptype != TYPE_TELEMETRY:
            del buf[0]
            continue

        payload = pkt[4:4 + PAYLOAD_LEN]
        calc_crc = packet_crc(length, ptype, payload)
        recv_crc = (pkt[-2] << 8) | pkt[-1]

        if calc_crc != recv_crc:
            del buf[0]
            continue

        del buf[:PACKET_SIZE]
        return decode_telemetry(payload)

    return None
