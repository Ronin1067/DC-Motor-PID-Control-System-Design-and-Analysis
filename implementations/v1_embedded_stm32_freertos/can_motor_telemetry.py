"""
ISO 11898-2 CAN-Bus Telemetry & Command Protocol for Precision Drive Dynamics.
Handles deterministic packing/unpacking of motor commands, current feedback,
and CRC-16 integrity validation.
"""

import struct
from typing import Dict, Optional, Tuple


def compute_crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT calculation for deterministic CAN telemetry frames."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class MotorCANTransceiver:
    """CAN-bus interface for 1 kHz drive telemetry and command dispatch."""

    MSG_CMD_ID = 0x280
    MSG_TELEMETRY_ID = 0x284

    def __init__(self, interface: str = "can0", bitrate: int = 1000000):
        self.interface = interface
        self.bitrate = bitrate

    def pack_velocity_command(self, target_vel_rad_s: float, torque_limit_nm: float) -> bytes:
        """
        Packs 16-bit target angular velocity and 16-bit torque limit with CRC-16.
        Format: [Vel_int16 (0.1 rad/s), Trq_uint16 (0.01 Nm), CRC16_uint16, Counter_uint8, Flags_uint8]
        """
        vel_raw = int(target_vel_rad_s * 10.0)
        trq_raw = int(min(max(torque_limit_nm, 0.0), 20.0) * 100.0)
        payload_no_crc = struct.pack(">hh", vel_raw, trq_raw)
        crc = compute_crc16_ccitt(payload_no_crc)
        return payload_no_crc + struct.pack(">HBB", crc, 0x01, 0x00)

    def unpack_telemetry(self, raw_bytes: bytes) -> Optional[Dict[str, float]]:
        """Unpacks and validates 8-byte CAN telemetry frame from STM32."""
        if len(raw_bytes) != 8:
            return None

        payload_to_verify = raw_bytes[:4]
        crc_received = struct.unpack(">H", raw_bytes[4:6])[0]
        if compute_crc16_ccitt(payload_to_verify) != crc_received:
            # Checksum mismatch
            return None

        vel_raw, cur_raw = struct.unpack(">hh", payload_to_verify)
        temp_raw, flags = struct.unpack(">BB", raw_bytes[6:8])

        return {
            "velocity_rad_s": vel_raw / 10.0,
            "current_a": cur_raw / 100.0,
            "winding_temp_c": float(temp_raw),
            "fault_flag": bool(flags & 0x01),
        }
