"""MID-360 read-only clock query codec, using the official little-endian protocol."""

from __future__ import annotations

import binascii
import struct
import zlib


CLOCK_KEYS = {
    0x8002: ("firmware", "4B"),
    0x8006: ("work_state", "B"),
    0x8008: ("powerup_count", "I"),
    0x8009: ("local_time_ns", "Q"),
    0x800A: ("last_sync_time_ns", "Q"),
    0x800B: ("time_offset_ns", "q"),
    0x800C: ("sync_type", "B"),
    0x800E: ("error_code", "H"),
}


def query_clock_packet(sequence: int) -> bytes:
    payload = struct.pack("<HH", len(CLOCK_KEYS), 0)
    payload += struct.pack("<" + "H" * len(CLOCK_KEYS), *CLOCK_KEYS)
    header = struct.pack("<BBHIHBB6s", 0xAA, 0, 24 + len(payload), sequence,
                         0x0101, 0, 0, bytes(6))
    return header + struct.pack("<HI", binascii.crc_hqx(header, 0xFFFF),
                                zlib.crc32(payload)) + payload


def parse_clock_response(packet: bytes, sequence: int) -> dict:
    if len(packet) < 27:
        raise ValueError("short_clock_response")
    sof, version, length, seq, command, kind, sender = struct.unpack_from("<BBHIHBB", packet)
    if (sof, version, length, seq, command, kind, sender) != (
            0xAA, 0, len(packet), sequence, 0x0101, 1, 1):
        raise ValueError("clock_response_header")
    crc16, crc32 = struct.unpack_from("<HI", packet, 18)
    if crc16 != binascii.crc_hqx(packet[:18], 0xFFFF) or crc32 != zlib.crc32(packet[24:]):
        raise ValueError("clock_response_crc")
    return_code, count = struct.unpack_from("<BH", packet, 24)
    if return_code:
        raise ValueError(f"clock_query_return_code:{return_code}")
    fields = {}
    cursor = 27
    for _ in range(count):
        if cursor + 4 > len(packet):
            raise ValueError("truncated_clock_key")
        key, size = struct.unpack_from("<HH", packet, cursor)
        cursor += 4
        if cursor + size > len(packet):
            raise ValueError("truncated_clock_value")
        if key in CLOCK_KEYS:
            name, fmt = CLOCK_KEYS[key]
            if size != struct.calcsize("<" + fmt):
                raise ValueError(f"clock_key_size:{key:04x}")
            value = struct.unpack_from("<" + fmt, packet, cursor)
            fields[name] = list(value) if name == "firmware" else value[0]
        cursor += size
    if cursor != len(packet):
        raise ValueError("clock_response_trailing_data")
    required = {name for name, _ in CLOCK_KEYS.values()}
    if missing := required - fields.keys():
        raise ValueError("clock_keys_missing:" + ",".join(sorted(missing)))
    return fields


def clock_evidence_reasons(driver: dict | None, device: dict | None,
                           now_utc_ns: int, utc_offset_s: int,
                           maximum_sync_age_s: float = 3.0) -> list[str]:
    """Mode/freshness evidence, not a bound on inter-sensor synchronization error."""
    reasons = []
    if driver is None:
        reasons.append("packet_clock_unavailable")
    else:
        if driver.get("fusion_reset_required", False):
            reasons.append("clock_epoch_changed_restart_lio")
        if driver["utc_offset_s"] != utc_offset_s:
            reasons.append("driver_utc_offset_mismatch")
        if not driver["require_ptp"]:
            reasons.append("driver_ptp_ingress_not_required")
        if not driver.get("device_clock_gate", False):
            reasons.append("driver_device_clock_gate_closed")
        for name in ("lidar", "imu"):
            stream = driver[name]
            if stream["time_type"] != 1:
                reasons.append(name + "_packet_not_ptp")
            if stream["packets"] == 0 or not 0 <= stream["receive_age_s"] <= 0.3:
                reasons.append(name + "_packets_stale")
    if device is None:
        reasons.append("device_clock_unavailable")
        return reasons
    if not device.get("valid", False):
        reasons.append("device_clock_query_failed")
        return reasons
    if not 0 <= (now_utc_ns - device["received_utc_ns"]) * 1e-9 <= 2.5:
        reasons.append("device_clock_status_stale")
    if device["sync_type"] != 1:
        reasons.append("device_sync_not_ptp")
    sync_age_s = (now_utc_ns - device["last_sync_time_ns"] + utc_offset_s * 10**9) * 1e-9
    if device["last_sync_time_ns"] == 0 or not -0.05 <= sync_age_s <= maximum_sync_age_s:
        reasons.append("last_ptp_sync_stale_or_future")
    if device["work_state"] != 1:
        reasons.append("device_not_sampling")
    if any((device["error_code"] >> shift & 0xF) >= 2 for shift in (0, 4, 8, 12)):
        reasons.append("device_error")
    # Protocol defines local minus source time. Its scale/meaning can vary with
    # firmware and clock mapping; do not claim a precision bound from this key.
    return reasons
