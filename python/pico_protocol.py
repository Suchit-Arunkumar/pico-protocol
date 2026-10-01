"""
pico_protocol.py
================
Pi-side implementation of the Pi <-> Pico 2 (RP2350) binary packet protocol.
Mirrors firmware/pico_protocol.h byte for byte.

Wire format:

    [ STX1 ][ STX2 ][ LEN ][ TYPE ][ PAYLOAD (56 bytes) ][ CRC_HI ][ CRC_LO ]
      0xAA    0x55    56     1B      little-endian          big-endian

    CRC-16/IBM-3740 (poly 0x1021, init 0xFFFF, no reflection, no final XOR)
    over [LEN, TYPE, PAYLOAD]. Total packet size: 62 bytes.

Packet types:
    0x01  TELEMETRY   Pico -> Pi   decoded by read_packet()
    0x02  CMD         Pi -> Pico   built by build_command_packet()
    0x03  PID         Pi -> Pico   built by build_pid_packets()

This module performs no I/O. It builds and parses byte buffers; wiring it to
a serial port is the caller's responsibility.
"""

import struct

# =============================================================================
# CONSTANTS  (mirror firmware/pico_protocol.h)
# =============================================================================

STX1 = 0xAA
STX2 = 0x55

# PACKET_SIZE is a literal, not derived: it is the on-wire truth that every
# other size is checked against below.
HEADER_SIZE = 4     # STX1 STX2 LEN TYPE
PAYLOAD_LEN = 56
CRC_SIZE    = 2     # CRC_HI CRC_LO
PACKET_SIZE = 62

TYPE_TELEMETRY = 0x01
TYPE_CMD       = 0x02
TYPE_PID       = 0x03

PROTOCOL_VERSION = 1
PID_NO_CHANGE    = -999.0


# =============================================================================
# WIRE LAYOUT
#
# Each field table is the single source of truth for its payload: the struct
# format string is derived from it, and the resulting byte offsets are
# asserted against the values pinned by _Static_assert(offsetof(...)) in the
# C header. A field reorder that preserves total size -- valid CRC, wrong
# values at the far end -- fails at import time here and at compile time
# there.
# =============================================================================

_TELEM_FIELDS = [
    ('depth_m',     'f'),
    ('raw_depth_m', 'f'),
    ('pid_u',       '6f'),
    ('esc_pwm',     '8H'),
    ('armed',       'B'),
    ('sat_flags',   'B'),
    ('link_ok',     'B'),
    ('version',     'B'),
    ('reserved',    '4s'),
]

_CMD_FIELDS = [
    ('current_x',    'f'), ('current_y',     'f'), ('current_z',   'f'),
    ('current_roll', 'f'), ('current_pitch', 'f'), ('current_yaw', 'f'),
    ('target_x',     'f'), ('target_y',      'f'), ('target_z',    'f'),
    ('target_roll',  'f'), ('target_pitch',  'f'), ('target_yaw',  'f'),
    ('armed',        'B'),
    ('seq',          'B'),
    ('version',      'B'),
    ('reserved',     '5s'),
]

_PID_FIELDS = [
    ('page',      'B'),
    ('version',   'B'),
    ('txn_id',    'B'),
    ('reserved0', 'B'),
    ('gains_a',   '6f'),
    ('gains_b',   '6f'),
    ('tail',      '4s'),
]


def _require(condition: bool, message: str) -> None:
    """Layout check that survives `python -O` (unlike a bare `assert`)."""
    if not condition:
        raise AssertionError(f'pico_protocol wire layout: {message}')


def _build_format(fields) -> str:
    """Little-endian, no padding -- matches __attribute__((packed))."""
    return '<' + ''.join(code for _, code in fields)


def _field_offsets(fields) -> dict:
    offsets, pos = {}, 0
    for name, code in fields:
        offsets[name] = pos
        pos += struct.calcsize('<' + code)
    return offsets


TELEM_FMT = _build_format(_TELEM_FIELDS)
CMD_FMT   = _build_format(_CMD_FIELDS)
PID_FMT   = _build_format(_PID_FIELDS)

TELEM_OFFSETS = _field_offsets(_TELEM_FIELDS)
CMD_OFFSETS   = _field_offsets(_CMD_FIELDS)
PID_OFFSETS   = _field_offsets(_PID_FIELDS)

_require(HEADER_SIZE + PAYLOAD_LEN + CRC_SIZE == PACKET_SIZE,
         'framing constants do not sum to PACKET_SIZE')
for _name, _fmt in (('TELEMETRY', TELEM_FMT), ('CMD', CMD_FMT), ('PID', PID_FMT)):
    _require(struct.calcsize(_fmt) == PAYLOAD_LEN,
             f'{_name} format is {struct.calcsize(_fmt)} bytes, not {PAYLOAD_LEN}')

# Offsets duplicated from the C _Static_asserts -- must match exactly.
_require(TELEM_OFFSETS == {
    'depth_m': 0, 'raw_depth_m': 4, 'pid_u': 8, 'esc_pwm': 32,
    'armed': 48, 'sat_flags': 49, 'link_ok': 50, 'version': 51, 'reserved': 52,
}, f'TELEMETRY offsets disagree with TelemetryPayload: {TELEM_OFFSETS}')

_require(CMD_OFFSETS == {
    'current_x': 0, 'current_y': 4, 'current_z': 8,
    'current_roll': 12, 'current_pitch': 16, 'current_yaw': 20,
    'target_x': 24, 'target_y': 28, 'target_z': 32,
    'target_roll': 36, 'target_pitch': 40, 'target_yaw': 44,
    'armed': 48, 'seq': 49, 'version': 50, 'reserved': 51,
}, f'CMD offsets disagree with CommandPayload: {CMD_OFFSETS}')

_require(PID_OFFSETS == {
    'page': 0, 'version': 1, 'txn_id': 2, 'reserved0': 3,
    'gains_a': 4, 'gains_b': 28, 'tail': 52,
}, f'PID offsets disagree with PidPayload: {PID_OFFSETS}')


# =============================================================================
# CRC-16/IBM-3740
# =============================================================================

def crc16_ccitt(data: bytes) -> int:
    """CRC-16/IBM-3740 (a.k.a. CCITT-FALSE). Check value of b'123456789' is 0x29B1."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def packet_crc(length: int, ptype: int, payload: bytes) -> int:
    """CRC over [LEN, TYPE, PAYLOAD]; the sync bytes are excluded."""
    return crc16_ccitt(bytes([length, ptype]) + payload)


def frame(ptype: int, payload: bytes) -> bytes:
    """Wrap a 56-byte payload in sync bytes, header and CRC."""
    _require(len(payload) == PAYLOAD_LEN, f'payload is {len(payload)} bytes')
    crc = packet_crc(PAYLOAD_LEN, ptype, payload)
    return (bytes([STX1, STX2, PAYLOAD_LEN, ptype])
            + payload
            + bytes([crc >> 8, crc & 0xFF]))


# =============================================================================
# ENCODE -- CMD (Pi -> Pico)
# =============================================================================

_seq = 0


def build_command_packet(pose: dict, target: dict, armed: int = 0, seq: int = None) -> bytes:
    """
    Build a TYPE_CMD packet.

    pose / target: dicts with keys x, y, z, roll, pitch, yaw.
    seq: explicit sequence number (0-255), or None to auto-increment.
    """
    global _seq
    if seq is None:
        _seq = (_seq + 1) & 0xFF
        seq = _seq

    payload = struct.pack(
        CMD_FMT,
        pose['x'], pose['y'], pose['z'],
        pose['roll'], pose['pitch'], pose['yaw'],
        target['x'], target['y'], target['z'],
        target['roll'], target['pitch'], target['yaw'],
        armed & 0xFF, seq & 0xFF,
        PROTOCOL_VERSION,
        b'\x00' * 5,
    )
    return frame(TYPE_CMD, payload)


# =============================================================================
# ENCODE -- PID (Pi -> Pico)
# =============================================================================

def build_pid_packets(kp=None, ki=None, kd=None, kff=None, txn_id: int = 0) -> list:
    """
    Build the TYPE_PID packets for a gain update.

    Each argument is a 6-element sequence (surge, sway, heave, roll, pitch,
    yaw) or None. Any element equal to PID_NO_CHANGE -- and every element of
    an argument left as None -- tells the firmware to leave that gain alone.

    Returns a list of 1 or 2 packets: page 0 (kp, ki) if either is given,
    page 1 (kd, kff) if either is given. When both pages are sent they share
    txn_id, so a receiver using pp_pid_txn_apply_page() applies them as one
    unit. Pass a distinct txn_id (1-255) per retune.
    """
    def gains(values):
        if values is None:
            return [PID_NO_CHANGE] * 6
        values = list(values)
        _require(len(values) == 6, f'gain vector has {len(values)} elements, not 6')
        return values

    packets = []
    for page, a, b in ((0, kp, ki), (1, kd, kff)):
        if a is None and b is None:
            continue
        payload = struct.pack(
            PID_FMT,
            page, PROTOCOL_VERSION, txn_id & 0xFF, 0,
            *gains(a), *gains(b),
            b'\x00' * 4,
        )
        packets.append(frame(TYPE_PID, payload))
    return packets


# =============================================================================
# DECODE -- TELEMETRY (Pico -> Pi)
# =============================================================================

def decode_telemetry(payload: bytes) -> dict:
    """Decode a 56-byte, already CRC-verified TELEMETRY payload."""
    v = struct.unpack(TELEM_FMT, payload)
    return {
        'depth_m':     v[0],         # fused depth echoed back -- do not feed
                                      # into the Pi's own estimator
        'raw_depth_m': v[1],         # onboard pressure sensor
        'pid_u':       list(v[2:8]),
        'esc_pwm':     list(v[8:16]),
        'armed':       v[16],
        'sat_flags':   v[17],        # bit0 vertical, bit1 horizontal, bit2 yaw
        'link_ok':     v[18],
        'version':     v[19],
    }


def read_packet(buf: bytearray):
    """
    Extract one valid TELEMETRY packet from the front of `buf`.

    Returns the decoded dict, or None if `buf` does not yet hold a complete
    valid packet. Uses the same three-check resync as the firmware parser:
    on a bad sync pair, LEN/TYPE, or CRC, exactly one byte is dropped and
    the search restarts, so a corrupted stream recovers within one packet
    width. `buf` is mutated in place.
    """
    while len(buf) >= PACKET_SIZE:
        if buf[0] != STX1 or buf[1] != STX2:
            del buf[0]
            continue

        length, ptype = buf[2], buf[3]
        if length != PAYLOAD_LEN or ptype != TYPE_TELEMETRY:
            del buf[0]
            continue

        payload = bytes(buf[4:4 + PAYLOAD_LEN])
        recv_crc = (buf[4 + PAYLOAD_LEN] << 8) | buf[4 + PAYLOAD_LEN + 1]
        if packet_crc(length, ptype, payload) != recv_crc:
            del buf[0]
            continue

        del buf[:PACKET_SIZE]
        return decode_telemetry(payload)

    return None
