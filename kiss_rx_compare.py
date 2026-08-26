#!/usr/bin/env python3
"""
Compare LoRa reception between two MeshCore KISS-modem radios.

Connects to both serial devices, configures each for the MeshCore
USA/Canada preset (910.525 MHz / 62.5 kHz BW / SF7 / CR5, TX power
set to 0 dBm for close-range bench testing), and logs every received
packet from both radios side by side (payload + RSSI/SNR/timing) so
their reception can be compared.

Requires: pyserial (pip install pyserial)
"""

import argparse
import json
import os
import signal
import struct
import sys
import threading
import time
from collections import Counter, defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import serial
from serial.tools import list_ports

# --- KISS framing ---------------------------------------------------------

KISS_FEND = 0xC0
KISS_FESC = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

KISS_CMD_DATA = 0x00
KISS_CMD_SETHARDWARE = 0x06
KISS_CMD_RETURN = 0xFF

# --- Hardware sub-commands -------------------------------------------------

HW_CMD_SET_RADIO = 0x09
HW_CMD_SET_TX_POWER = 0x0A
HW_CMD_GET_RADIO = 0x0B
HW_CMD_GET_CURRENT_RSSI = 0x0D
HW_CMD_GET_NOISE_FLOOR = 0x10
HW_CMD_GET_VERSION = 0x11
HW_CMD_GET_STATS = 0x12
HW_CMD_GET_BATTERY = 0x13
HW_CMD_GET_DEVICE_NAME = 0x16
HW_CMD_PING = 0x17

HW_RESP_OK = 0xF0
HW_RESP_ERROR = 0xF1
HW_RESP_TX_DONE = 0xF8
HW_RESP_RX_META = 0xF9


def hw_resp(cmd):
    return cmd | 0x80


# US/Canada preset (MeshCore "narrow" recommended default as of Oct 2025).
US_FREQ_HZ = 910_525_000
US_BW_HZ = 62_500
US_SF = 7
US_CR = 5
US_TX_POWER_DBM = 0

READ_TIMEOUT_S = 0.2
STARTUP_RESPONSE_TIMEOUT_S = 2.0
STATS_POLL_INTERVAL_S = 5.0
DEFAULT_ROLLING_WINDOW = 20
DEFAULT_MATCH_WINDOW_S = 3.0
DEFAULT_SUMMARY_INTERVAL_S = 30.0

# --- MeshCore on-air packet parsing (best-effort, cleartext fields only) ---
#
# Layout below is derived from MeshCore's src/Packet.{h,cpp}, src/Mesh.cpp and
# src/helpers/AdvertDataHelpers.{h,cpp}. This tool has no shared secrets, so it
# can only decode the unencrypted parts of a packet: the routing header, the
# path, and (for ADVERT only) the full application payload, which is sent
# unencrypted and self-signed. Everything else exposes at most a 1-byte
# dest/src/channel "hash" (the first byte of a node's 32-byte public key) plus
# an opaque encrypted blob.

ROUTE_TYPE_NAMES = {
    0x00: "XPORT_FLOOD",
    0x01: "FLOOD",
    0x02: "DIRECT",
    0x03: "XPORT_DIRECT",
}

PAYLOAD_TYPE_NAMES = {
    0x00: "REQ",
    0x01: "RESPONSE",
    0x02: "TXT_MSG",
    0x03: "ACK",
    0x04: "ADVERT",
    0x05: "GRP_TXT",
    0x06: "GRP_DATA",
    0x07: "ANON_REQ",
    0x08: "PATH",
    0x09: "TRACE",
    0x0A: "MULTIPART",
    0x0B: "CONTROL",
    0x0F: "RAW_CUSTOM",
}

ADV_TYPE_NAMES = {0: "NONE", 1: "CHAT", 2: "REPEATER", 3: "ROOM", 4: "SENSOR"}

MC_PUB_KEY_SIZE = 32
MC_SIGNATURE_SIZE = 64
MC_CIPHER_MAC_SIZE = 2


def decode_mc_packet(payload: bytes) -> dict:
    """Best-effort decode of a MeshCore packet's cleartext fields.

    Never raises: firmware already validated framing before this payload ever
    reached us, but our own parsing here is speculative (there's no length
    field to cross-check against), so any short/malformed data just yields a
    partial result instead of blowing up the reader thread.
    """
    info = {"total_len": len(payload)}
    if not payload:
        return info
    try:
        header = payload[0]
        route_type = header & 0x03
        payload_type = (header >> 2) & 0x0F
        info["route_type"] = ROUTE_TYPE_NAMES.get(route_type, f"0x{route_type:02X}")
        info["payload_type"] = PAYLOAD_TYPE_NAMES.get(payload_type, f"0x{payload_type:02X}")
        info["payload_ver"] = (header >> 6) & 0x03

        i = 1
        if route_type in (0x00, 0x03):  # has transport codes
            i += 4

        path_len_byte = payload[i]
        i += 1
        hash_size = (path_len_byte >> 6) + 1
        hash_count = path_len_byte & 0x3F
        path_bytes_len = hash_count * hash_size
        path_bytes = payload[i : i + path_bytes_len]
        i += path_bytes_len
        info["hop_count"] = hash_count
        if hash_size == 1 and path_bytes:
            info["path_hashes"] = path_bytes.hex()

        app_payload = payload[i:]
        _decode_mc_app_payload(payload_type, app_payload, info)
    except (IndexError, struct.error):
        pass
    return info


def _decode_mc_app_payload(payload_type: int, data: bytes, info: dict):
    if payload_type == 0x04:  # ADVERT -- fully cleartext & self-signed
        if len(data) < MC_PUB_KEY_SIZE + 4 + MC_SIGNATURE_SIZE:
            return
        pub_key = data[:MC_PUB_KEY_SIZE]
        info["node_hash"] = f"{pub_key[0]:02x}"
        info["pub_key_prefix"] = pub_key[:4].hex()
        info["advert_timestamp"] = struct.unpack("<I", data[MC_PUB_KEY_SIZE : MC_PUB_KEY_SIZE + 4])[0]
        app_data = data[MC_PUB_KEY_SIZE + 4 + MC_SIGNATURE_SIZE :]
        if not app_data:
            return
        flags = app_data[0]
        j = 1
        info["adv_type"] = ADV_TYPE_NAMES.get(flags & 0x0F, f"0x{flags & 0x0F:x}")
        if flags & 0x10:  # lat/lon
            if len(app_data) >= j + 8:
                info["lat"] = struct.unpack("<i", app_data[j : j + 4])[0] / 1e6
                info["lon"] = struct.unpack("<i", app_data[j + 4 : j + 8])[0] / 1e6
            j += 8
        if flags & 0x20:
            j += 2
        if flags & 0x40:
            j += 2
        if flags & 0x80 and len(app_data) > j:
            info["name"] = app_data[j:].decode("utf-8", errors="replace")
    elif payload_type in (0x00, 0x01, 0x02, 0x08):  # REQ, RESPONSE, TXT_MSG, PATH
        if len(data) < 2 + MC_CIPHER_MAC_SIZE:
            return
        info["dest_hash"] = f"{data[0]:02x}"
        info["src_hash"] = f"{data[1]:02x}"
        info["encrypted_len"] = len(data) - 2 - MC_CIPHER_MAC_SIZE
    elif payload_type == 0x03:  # ACK -- opaque CRC, no hash prefix
        if len(data) >= 4:
            info["ack_crc"] = struct.unpack("<I", data[:4])[0]
    elif payload_type in (0x05, 0x06):  # GRP_TXT, GRP_DATA
        if len(data) < 1 + MC_CIPHER_MAC_SIZE:
            return
        info["channel_hash"] = f"{data[0]:02x}"
        info["encrypted_len"] = len(data) - 1 - MC_CIPHER_MAC_SIZE
    elif payload_type == 0x07:  # ANON_REQ
        if len(data) < 1 + MC_PUB_KEY_SIZE + MC_CIPHER_MAC_SIZE:
            return
        info["dest_hash"] = f"{data[0]:02x}"
        info["sender_pub_key_prefix"] = data[1:5].hex()
    elif payload_type == 0x0A:  # MULTIPART
        if data:
            info["multipart_remaining"] = data[0] >> 4
            inner_type = data[0] & 0x0F
            info["multipart_inner_type"] = PAYLOAD_TYPE_NAMES.get(inner_type, f"0x{inner_type:x}")
    elif payload_type == 0x09:  # TRACE -- cleartext hop tracing
        if len(data) >= 4:
            info["trace_tag"] = struct.unpack("<I", data[:4])[0]


def describe_mc_packet(mc: dict) -> str:
    """One-line human summary of a decode_mc_packet() result, for console/log display."""
    if not mc:
        return ""
    ptype = mc.get("payload_type", "?")
    bits = []
    if ptype == "ADVERT":
        if "name" in mc:
            bits.append(f'name="{mc["name"]}"')
        if "adv_type" in mc:
            bits.append(f"adv_type={mc['adv_type']}")
        if "node_hash" in mc:
            bits.append(f"node={mc['node_hash']}")
    elif ptype in ("REQ", "RESPONSE", "TXT_MSG", "PATH"):
        bits.append(f"dest={mc.get('dest_hash')} src={mc.get('src_hash')}")
    elif ptype == "ACK":
        if "ack_crc" in mc:
            bits.append(f"crc=0x{mc['ack_crc']:08x}")
    elif ptype in ("GRP_TXT", "GRP_DATA"):
        bits.append(f"chan={mc.get('channel_hash')}")
    elif ptype == "ANON_REQ":
        bits.append(f"dest={mc.get('dest_hash')}")
    extra = (" " + " ".join(bits)) if bits else ""
    return f"type={ptype}{extra}"


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def encode_kiss_frame(cmd, payload=b""):
    """Encode a single KISS frame: FEND, escaped(type||payload), FEND. Port 0 always."""
    out = bytearray([KISS_FEND])
    for b in bytes([cmd]) + bytes(payload):
        if b == KISS_FEND:
            out += bytes([KISS_FESC, KISS_TFEND])
        elif b == KISS_FESC:
            out += bytes([KISS_FESC, KISS_TFESC])
        else:
            out.append(b)
    out.append(KISS_FEND)
    return bytes(out)


def encode_hw_frame(sub_cmd, payload=b""):
    return encode_kiss_frame(KISS_CMD_SETHARDWARE, bytes([sub_cmd]) + bytes(payload))


class KissDecoder:
    """Incremental KISS de-framer/de-escaper. Feed raw bytes, get back complete frames."""

    def __init__(self):
        self._buf = bytearray()
        self._escaped = False
        self._active = False

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if b == KISS_FEND:
                if self._active and self._buf:
                    frames.append(bytes(self._buf))
                self._buf = bytearray()
                self._escaped = False
                self._active = True
                continue
            if not self._active:
                continue
            if b == KISS_FESC:
                self._escaped = True
                continue
            if self._escaped:
                self._escaped = False
                if b == KISS_TFEND:
                    b = KISS_FEND
                elif b == KISS_TFESC:
                    b = KISS_FESC
                else:
                    continue
            self._buf.append(b)
        return frames


@dataclass
class PendingPacket:
    payload: bytes
    recv_time: float


class RunningStat:
    """All-time count/mean/min/max for a stream of numbers."""

    def __init__(self):
        self.count = 0
        self.total = 0.0
        self.minimum = None
        self.maximum = None

    def add(self, x: float):
        self.count += 1
        self.total += x
        self.minimum = x if self.minimum is None else min(self.minimum, x)
        self.maximum = x if self.maximum is None else max(self.maximum, x)

    @property
    def mean(self):
        return self.total / self.count if self.count else None


def rolling_mean(values) -> "float | None":
    values = list(values)
    return sum(values) / len(values) if values else None


@dataclass
class RadioLink:
    label: str
    port: str
    ser: serial.Serial
    decoder: KissDecoder = field(default_factory=KissDecoder)
    pending: "PendingPacket | None" = None
    packet_count: int = 0
    log_fh: object = None
    last_stats: "tuple[int, int, int] | None" = None
    decode_error_total: int = 0
    meta_only_count: int = 0
    no_meta_count: int = 0
    payload_type_counts: Counter = field(default_factory=Counter)
    rssi_stats: RunningStat = field(default_factory=RunningStat)
    snr_stats: RunningStat = field(default_factory=RunningStat)
    rolling_rssi: deque = field(default_factory=lambda: deque(maxlen=DEFAULT_ROLLING_WINDOW))
    rolling_snr: deque = field(default_factory=lambda: deque(maxlen=DEFAULT_ROLLING_WINDOW))


class CrossRadioMatcher:
    """Correlates packets seen on both radios so we can tell 'both heard this'
    apart from 'only one radio heard this'.

    Two radios sitting side by side receiving the same over-the-air
    transmission decode identical payload bytes, so raw payload bytes (within
    a short time window) are used as the correlation key -- no need to
    recompute MeshCore's own packet hash or hold any keys.
    """

    def __init__(self, window_s: float = DEFAULT_MATCH_WINDOW_S, rolling_window: int = DEFAULT_ROLLING_WINDOW):
        self.window_s = window_s
        self.lock = threading.Lock()
        self.pending = {}  # payload bytes -> {"first_ts":, "seen": {label: (rssi, snr, mc)}}
        self.order = deque()  # (first_ts, payload) in observation order
        self.both_count = 0
        self.unique_count = defaultdict(int)  # label -> count
        self.delta_rssi = RunningStat()  # first_label - second_label, alphabetically
        self.delta_snr = RunningStat()
        self.rolling_delta_rssi = deque(maxlen=rolling_window)
        self.rolling_delta_snr = deque(maxlen=rolling_window)

    def observe(self, label: str, payload: bytes, ts: float, rssi: int, snr: float, mc: dict):
        with self.lock:
            entry = self.pending.get(payload)
            if entry is None:
                entry = {"first_ts": ts, "seen": {}}
                self.pending[payload] = entry
                self.order.append((ts, payload))
            entry["seen"][label] = (rssi, snr, mc)

    def _finalize(self, entry: dict) -> "tuple[str, dict] | None":
        """Record one expired entry's stats. Returns (label, seen_tuple) if it
        was heard by exactly one radio, so the caller can log a unique-rx event."""
        seen = entry["seen"]
        if len(seen) >= 2:
            labels = sorted(seen)
            rssi_a, snr_a, _ = seen[labels[0]]
            rssi_b, snr_b, _ = seen[labels[1]]
            d_rssi, d_snr = rssi_a - rssi_b, snr_a - snr_b
            self.both_count += 1
            self.delta_rssi.add(d_rssi)
            self.delta_snr.add(d_snr)
            self.rolling_delta_rssi.append(d_rssi)
            self.rolling_delta_snr.append(d_snr)
            return None
        (label, seen_tuple) = next(iter(seen.items()))
        self.unique_count[label] += 1
        return label, seen_tuple

    def sweep(self) -> list:
        """Finalize entries whose match window has expired. Returns a list of
        (label, rssi, snr, mc) for packets that turned out unique to one radio."""
        now = time.time()
        expired = []
        with self.lock:
            while self.order and now - self.order[0][0] > self.window_s:
                _, key = self.order.popleft()
                entry = self.pending.pop(key, None)
                if entry is not None:
                    expired.append(entry)
            results = []
            for entry in expired:
                r = self._finalize(entry)
                if r is not None:
                    label, (rssi, snr, mc) = r
                    results.append((label, rssi, snr, mc))
            return results

    def flush_all(self) -> list:
        """Finalize every still-pending entry regardless of age (used at shutdown)."""
        with self.lock:
            entries = list(self.pending.values())
            self.pending.clear()
            self.order.clear()
            results = []
            for entry in entries:
                r = self._finalize(entry)
                if r is not None:
                    label, (rssi, snr, mc) = r
                    results.append((label, rssi, snr, mc))
            return results


class NodeTable:
    """Tracks distinct MeshCore nodes seen via their (cleartext, self-signed)
    ADVERT packets, and which radio(s) hear each one -- useful for spotting
    repeaters/nodes that only one radio can hear."""

    def __init__(self):
        self.lock = threading.Lock()
        self.nodes = {}  # node_hash -> {"name":, "adv_type":, "per_radio": {label: {...}}}

    def observe(self, label: str, node_hash: str, name: "str | None", adv_type: "str | None", rssi: int, snr: float):
        with self.lock:
            rec = self.nodes.setdefault(node_hash, {"name": None, "adv_type": None, "per_radio": {}})
            if name:
                rec["name"] = name
            if adv_type:
                rec["adv_type"] = adv_type
            pr = rec["per_radio"].setdefault(label, {"count": 0})
            pr["count"] += 1
            pr["last_rssi"] = rssi
            pr["last_snr"] = snr

    def snapshot(self) -> dict:
        with self.lock:
            return {h: {**rec, "per_radio": dict(rec["per_radio"])} for h, rec in self.nodes.items()}


def hexdump(data: bytes) -> str:
    return data.hex()


def printable_ascii(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b < 127 else "." for b in data)


def log_event(link: RadioLink, event: dict, print_lock: threading.Lock):
    event = {"time": now_iso(), "radio": link.label, "port": link.port, **event}
    line = json.dumps(event)
    with print_lock:
        print(format_console(event))
        if link.log_fh:
            link.log_fh.write(line + "\n")
            link.log_fh.flush()


def truncate_hex(hex_str: str, max_chars: int = 64) -> str:
    if len(hex_str) <= max_chars:
        return hex_str
    return f"{hex_str[:max_chars]}...(+{(len(hex_str) - max_chars) // 2}B)"


def format_console(event: dict) -> str:
    kind = event["type"]
    prefix = f"[{event['time']}] {event['radio']:>1} "
    if kind == "packet":
        mc_desc = describe_mc_packet(event.get("mc", {}))
        mc_part = f"{mc_desc:<28} " if mc_desc else ""
        return (
            f"{prefix}RX len={event['len']:<3} {mc_part}"
            f"rssi={event['rssi_dbm']:>4} dBm snr={event['snr_db']:>6.2f} dB  "
            f"payload={truncate_hex(event['payload_hex'])}"
        )
    if kind == "meta_only":
        return f"{prefix}META (no matching packet) rssi={event['rssi_dbm']} snr={event['snr_db']:.2f}"
    if kind == "packet_no_meta":
        return f"{prefix}RX len={event['len']:<3} (no meta received) payload={truncate_hex(event['payload_hex'])}"
    if kind == "unique_to_radio":
        mc_desc = describe_mc_packet(event.get("mc", {}))
        mc_part = f" {mc_desc}" if mc_desc else ""
        return (
            f"{prefix}UNIQUE RX (other radio missed this) len={event['len']:<3}{mc_part} "
            f"rssi={event['rssi_dbm']:>4} dBm snr={event['snr_db']:>6.2f} dB"
        )
    if kind == "decode_error":
        return f"{prefix}DECODE FAILED x{event['count']} (header/CRC detected but couldn't be decoded into a packet)"
    if kind == "hw_resp":
        return f"{prefix}HW resp sub_cmd=0x{event['sub_cmd']:02X} payload={event['payload_hex']}"
    if kind == "hw_error":
        return f"{prefix}HW ERROR code=0x{event['code']:02X}"
    if kind == "hw_ok":
        return f"{prefix}HW OK"
    if kind == "tx_done":
        return f"{prefix}TX done result=0x{event['result']:02X}"
    if kind == "info":
        return f"{prefix}{event['message']}"
    return f"{prefix}{event}"


def record_packet_stats(link: RadioLink, rssi_dbm: int, snr_db: float, mc: dict):
    link.rssi_stats.add(rssi_dbm)
    link.snr_stats.add(snr_db)
    link.rolling_rssi.append(rssi_dbm)
    link.rolling_snr.append(snr_db)
    link.payload_type_counts[mc.get("payload_type", "UNKNOWN")] += 1


def handle_frame(
    link: RadioLink,
    frame: bytes,
    print_lock: threading.Lock,
    matcher: "CrossRadioMatcher | None",
    node_table: "NodeTable | None",
):
    if len(frame) < 1:
        return
    type_byte = frame[0]
    if type_byte == KISS_CMD_RETURN:
        return
    port = (type_byte >> 4) & 0x0F
    cmd = type_byte & 0x0F
    data = frame[1:]
    if port != 0:
        return

    if cmd == KISS_CMD_DATA:
        if link.pending is not None:
            link.no_meta_count += 1
            log_event(
                link,
                {
                    "type": "packet_no_meta",
                    "len": len(link.pending.payload),
                    "payload_hex": hexdump(link.pending.payload),
                    "payload_ascii": printable_ascii(link.pending.payload),
                },
                print_lock,
            )
        link.pending = PendingPacket(payload=bytes(data), recv_time=time.time())
        return

    if cmd == KISS_CMD_SETHARDWARE:
        if len(data) < 1:
            return
        sub_cmd = data[0]
        payload = data[1:]

        if sub_cmd == HW_RESP_RX_META:
            snr_raw = struct.unpack("b", payload[0:1])[0] if len(payload) >= 1 else 0
            rssi_raw = struct.unpack("b", payload[1:2])[0] if len(payload) >= 2 else 0
            snr_db = snr_raw / 4.0
            rssi_dbm = rssi_raw
            if link.pending is not None:
                pkt = link.pending
                link.pending = None
                link.packet_count += 1
                mc = decode_mc_packet(pkt.payload)
                record_packet_stats(link, rssi_dbm, snr_db, mc)
                if mc.get("payload_type") == "ADVERT" and "node_hash" in mc and node_table is not None:
                    node_table.observe(
                        link.label, mc["node_hash"], mc.get("name"), mc.get("adv_type"), rssi_dbm, snr_db
                    )
                if matcher is not None:
                    matcher.observe(link.label, pkt.payload, pkt.recv_time, rssi_dbm, snr_db, mc)
                log_event(
                    link,
                    {
                        "type": "packet",
                        "seq": link.packet_count,
                        "len": len(pkt.payload),
                        "payload_hex": hexdump(pkt.payload),
                        "payload_ascii": printable_ascii(pkt.payload),
                        "rssi_dbm": rssi_dbm,
                        "snr_db": snr_db,
                        "mc": mc,
                    },
                    print_lock,
                )
            else:
                link.meta_only_count += 1
                log_event(link, {"type": "meta_only", "rssi_dbm": rssi_dbm, "snr_db": snr_db}, print_lock)
            return

        if sub_cmd == HW_RESP_ERROR:
            code = payload[0] if payload else 0
            log_event(link, {"type": "hw_error", "code": code}, print_lock)
            return

        if sub_cmd == HW_RESP_OK:
            log_event(link, {"type": "hw_ok"}, print_lock)
            return

        if sub_cmd == HW_RESP_TX_DONE:
            result = payload[0] if payload else 0
            log_event(link, {"type": "tx_done", "result": result}, print_lock)
            return

        if sub_cmd == hw_resp(HW_CMD_GET_STATS):
            if len(payload) >= 12:
                stats = struct.unpack("<III", payload[:12])
                _, _, errors = stats
                if link.last_stats is not None and errors > link.last_stats[2]:
                    delta = errors - link.last_stats[2]
                    link.decode_error_total += delta
                    log_event(link, {"type": "decode_error", "count": delta}, print_lock)
                link.last_stats = stats
            return

        log_event(link, {"type": "hw_resp", "sub_cmd": sub_cmd, "payload_hex": hexdump(payload)}, print_lock)


def reader_thread(
    link: RadioLink,
    print_lock: threading.Lock,
    stop_event: threading.Event,
    matcher: "CrossRadioMatcher | None" = None,
    node_table: "NodeTable | None" = None,
):
    while not stop_event.is_set():
        try:
            data = link.ser.read(4096)
        except serial.SerialException as e:
            log_event(link, {"type": "info", "message": f"serial error: {e}"}, print_lock)
            return
        if not data:
            continue
        for frame in link.decoder.feed(data):
            handle_frame(link, frame, print_lock, matcher, node_table)


def emit_unique_events(links_by_label: dict, results: list, print_lock: threading.Lock):
    """Log 'heard by only one radio' events discovered by a matcher sweep, into
    that radio's own log (console + JSONL)."""
    for label, rssi, snr, mc in results:
        link = links_by_label.get(label)
        if link is None:
            continue
        log_event(
            link,
            {
                "type": "unique_to_radio",
                "len": mc.get("total_len"),
                "rssi_dbm": rssi,
                "snr_db": snr,
                "mc": mc,
            },
            print_lock,
        )


def fmt_stat(stat: RunningStat, rolling: deque, unit: str, precision: int = 1) -> str:
    if stat.count == 0:
        return "n/a"
    roll = rolling_mean(rolling)
    roll_part = f" | last {len(rolling)}: avg {roll:.{precision}f}{unit}" if roll is not None else ""
    return (
        f"avg {stat.mean:.{precision}f}{unit} min {stat.minimum:.{precision}f}{unit} "
        f"max {stat.maximum:.{precision}f}{unit}{roll_part}"
    )


def render_summary(
    links: list, matcher: "CrossRadioMatcher | None", node_table: NodeTable, start_time: float
) -> str:
    elapsed = int(time.time() - start_time)
    lines = [f"\n=== Summary @ {now_iso()} (elapsed {elapsed // 60}m{elapsed % 60:02d}s) ==="]

    for link in links:
        types = ", ".join(f"{t}={c}" for t, c in link.payload_type_counts.most_common())
        lines.append(f"Radio {link.label} ({link.port}):")
        lines.append(
            f"  packets: {link.packet_count}   decode errors: {link.decode_error_total}   "
            f"framing mismatches: {link.meta_only_count + link.no_meta_count}"
        )
        if matcher is not None:
            lines.append(
                f"  heard by both radios: {matcher.both_count}   "
                f"unique to {link.label}: {matcher.unique_count.get(link.label, 0)}"
            )
        lines.append(f"  RSSI: {fmt_stat(link.rssi_stats, link.rolling_rssi, ' dBm', 1)}")
        lines.append(f"  SNR:  {fmt_stat(link.snr_stats, link.rolling_snr, ' dB', 2)}")
        if types:
            lines.append(f"  payload types: {types}")

    if matcher is not None:
        lines.append(f"Comparison (n={matcher.both_count} packets heard by both, delta = A - B):")
        lines.append(f"  ΔRSSI: {fmt_stat(matcher.delta_rssi, matcher.rolling_delta_rssi, ' dB', 1)}")
        lines.append(f"  ΔSNR:  {fmt_stat(matcher.delta_snr, matcher.rolling_delta_snr, ' dB', 2)}")

    nodes = node_table.snapshot()
    if nodes:
        lines.append(f"Nodes heard via ADVERT: {len(nodes)}")
        for node_hash, rec in sorted(nodes.items(), key=lambda kv: -sum(pr["count"] for pr in kv[1]["per_radio"].values())):
            name = rec["name"] or "(unnamed)"
            adv_type = rec["adv_type"] or "?"
            per_radio = ", ".join(
                f"{label}: {pr['count']} (last rssi={pr['last_rssi']} snr={pr['last_snr']:.1f})"
                for label, pr in sorted(rec["per_radio"].items())
            )
            lines.append(f"  {node_hash} {name!r:<24} [{adv_type:<9}] {per_radio}")

    lines.append("=" * 40)
    return "\n".join(lines)


def housekeeping_thread(
    links: list,
    matcher: "CrossRadioMatcher | None",
    node_table: NodeTable,
    stop_event: threading.Event,
    print_lock: threading.Lock,
    summary_interval: float,
    start_time: float,
):
    links_by_label = {link.label: link for link in links}
    next_summary = start_time + summary_interval if summary_interval > 0 else None
    while not stop_event.wait(0.5):
        if matcher is not None:
            emit_unique_events(links_by_label, matcher.sweep(), print_lock)
        if next_summary is not None and time.time() >= next_summary:
            with print_lock:
                print(render_summary(links, matcher, node_table, start_time))
            next_summary = time.time() + summary_interval


def stats_poll_thread(link: RadioLink, stop_event: threading.Event, interval=STATS_POLL_INTERVAL_S):
    """Periodically request GetStats so RX errors (packets whose header/CRC the
    radio detected but couldn't decode) show up in the log -- those never
    generate a Data/RxMeta frame on their own, so without this poll a failing
    receiver looks identical to a silent one."""
    while not stop_event.wait(interval):
        try:
            link.ser.write(encode_hw_frame(HW_CMD_GET_STATS))
        except serial.SerialException:
            return


def wait_for_hw_response(link: RadioLink, timeout=STARTUP_RESPONSE_TIMEOUT_S):
    """Read raw bytes directly (used only during single-threaded startup config, before reader threads start)."""
    decoder = KissDecoder()
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = link.ser.read(256)
        if not data:
            continue
        for frame in decoder.feed(data):
            if len(frame) >= 2 and (frame[0] & 0x0F) == KISS_CMD_SETHARDWARE:
                sub_cmd = frame[1]
                if sub_cmd == HW_RESP_OK:
                    return True, None
                if sub_cmd == HW_RESP_ERROR:
                    return False, frame[2] if len(frame) > 2 else None
    return False, "timeout"


def wait_for_reply(link: RadioLink, expected_sub_cmd: int, timeout=STARTUP_RESPONSE_TIMEOUT_S):
    """Read raw bytes directly, waiting for a specific SETHARDWARE response sub-command.
    Used only during single-threaded startup, before reader threads start."""
    decoder = KissDecoder()
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = link.ser.read(256)
        if not data:
            continue
        for frame in decoder.feed(data):
            if len(frame) >= 2 and (frame[0] & 0x0F) == KISS_CMD_SETHARDWARE:
                sub_cmd = frame[1]
                if sub_cmd == expected_sub_cmd:
                    return frame[2:]
                if sub_cmd == HW_RESP_ERROR:
                    return None
    return None


def run_radio_diagnostics(link: RadioLink, print_lock: threading.Lock):
    """Read back applied radio config and noise floor, to sanity-check that SET_RADIO
    actually took effect and that the receiver is picking up any RF energy at all."""
    link.ser.write(encode_hw_frame(HW_CMD_GET_RADIO))
    payload = wait_for_reply(link, hw_resp(HW_CMD_GET_RADIO))
    if payload is not None and len(payload) >= 10:
        freq_hz, bw_hz, sf, cr = struct.unpack("<IIBB", payload[:10])
        message = f"GET_RADIO readback: {freq_hz / 1e6:.3f} MHz, {bw_hz / 1e3:.1f} kHz, SF{sf}, CR{cr}"
    else:
        message = "GET_RADIO readback: FAILED (no response)"
    log_event(link, {"type": "info", "message": message}, print_lock)

    link.ser.write(encode_hw_frame(HW_CMD_GET_NOISE_FLOOR))
    payload = wait_for_reply(link, hw_resp(HW_CMD_GET_NOISE_FLOOR))
    if payload is not None and len(payload) >= 2:
        noise_dbm = struct.unpack("<h", payload[:2])[0]
        message = f"GET_NOISE_FLOOR: {noise_dbm} dBm"
    else:
        message = "GET_NOISE_FLOOR: FAILED (no response)"
    log_event(link, {"type": "info", "message": message}, print_lock)

    link.ser.write(encode_hw_frame(HW_CMD_GET_BATTERY))
    payload = wait_for_reply(link, hw_resp(HW_CMD_GET_BATTERY))
    if payload is not None and len(payload) >= 2:
        millivolts = struct.unpack("<H", payload[:2])[0]
        message = f"GET_BATTERY: {millivolts / 1000.0:.2f} V"
    else:
        message = "GET_BATTERY: FAILED (no response)"
    log_event(link, {"type": "info", "message": message}, print_lock)


def rssi_monitor(link: RadioLink, interval=0.2):
    """Continuously poll GetCurrentRssi and print it live. This reads the raw
    RF front-end power level directly -- it moves on any energy the radio
    front-end picks up, whether or not MeshCore can decode a packet out of it.
    Use this to tell 'RF chain isn't detecting anything' (RSSI never moves,
    hints at antenna/RF hardware) apart from 'detects energy but can't decode
    it' (RSSI moves fine, but --no packets, check GetStats error counter)."""
    print(f"\nLive RSSI monitor for radio {link.label} ({link.port}). Press Ctrl+C to stop.")
    print("Trigger a transmission on the other device nearby and watch for the value to jump up (less negative).\n")
    decoder = KissDecoder()
    try:
        while True:
            link.ser.write(encode_hw_frame(HW_CMD_GET_CURRENT_RSSI))
            deadline = time.time() + 1.0
            rssi = None
            while time.time() < deadline and rssi is None:
                data = link.ser.read(64)
                if not data:
                    continue
                for frame in decoder.feed(data):
                    if len(frame) >= 3 and (frame[0] & 0x0F) == KISS_CMD_SETHARDWARE and frame[1] == hw_resp(
                        HW_CMD_GET_CURRENT_RSSI
                    ):
                        rssi = struct.unpack("b", frame[2:3])[0]
                        break
            ts = now_iso()
            if rssi is not None:
                bar = "#" * max(0, min(60, rssi + 130))
                print(f"\r[{ts}] RSSI: {rssi:>4} dBm {bar:<60}", end="", flush=True)
            else:
                print(f"\r[{ts}] RSSI: no response (timeout){' ' * 40}", end="", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print()


def configure_us_defaults(link: RadioLink, print_lock: threading.Lock):
    radio_payload = struct.pack("<IIBB", US_FREQ_HZ, US_BW_HZ, US_SF, US_CR)
    link.ser.write(encode_hw_frame(HW_CMD_SET_RADIO, radio_payload))
    ok, err = wait_for_hw_response(link)
    log_event(
        link,
        {
            "type": "info",
            "message": f"SET_RADIO({US_FREQ_HZ / 1e6:.3f} MHz, {US_BW_HZ / 1e3:.1f} kHz, SF{US_SF}, CR{US_CR}) "
            + ("OK" if ok else f"FAILED ({err})"),
        },
        print_lock,
    )

    link.ser.write(encode_hw_frame(HW_CMD_SET_TX_POWER, bytes([US_TX_POWER_DBM])))
    ok, err = wait_for_hw_response(link)
    log_event(
        link,
        {"type": "info", "message": f"SET_TX_POWER({US_TX_POWER_DBM} dBm) " + ("OK" if ok else f"FAILED ({err})")},
        print_lock,
    )


# --- Port discovery ---------------------------------------------------------

BY_ID_DIR = Path("/dev/serial/by-id")


def by_id_for_device(dev_path: str) -> str | None:
    """Return the /dev/serial/by-id/* symlink pointing at dev_path, if any."""
    if not BY_ID_DIR.is_dir():
        return None
    try:
        target = Path(dev_path).resolve()
    except OSError:
        return None
    for link in BY_ID_DIR.iterdir():
        try:
            if link.resolve() == target:
                return str(link)
        except OSError:
            continue
    return None


def is_phantom_legacy_port(p) -> bool:
    """Filter out /dev/ttyS* legacy platform UARTs that aren't backed by real hardware."""
    return p.vid is None and Path(p.device).name.startswith("ttyS")


def discover_ports():
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    ports = [p for p in ports if not is_phantom_legacy_port(p)]
    for p in ports:
        by_id = by_id_for_device(p.device)
        if by_id:
            p.device = by_id
    return ports


def prompt_choice(ports, label):
    print(f"\nSelect serial port for radio {label}:")
    for i, p in enumerate(ports):
        desc = f" - {p.description}" if p.description else ""
        print(f"  [{i}] {p.device}{desc}")
    while True:
        choice = input(f"Enter index for radio {label}: ").strip()
        try:
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        print("Invalid selection, try again.")


def select_ports(explicit_a, explicit_b, single=False):
    """Returns (port_a, port_b). port_b is None when running in single-radio mode."""
    if explicit_a and explicit_b:
        return explicit_a, explicit_b

    ports = discover_ports()

    if single:
        if explicit_a:
            return explicit_a, None
        if len(ports) == 1:
            return ports[0].device, None
        if not ports:
            print("No serial ports found. Plug in the radio and retry, or pass --port-a explicitly.")
            sys.exit(1)
        return prompt_choice(ports, "A"), None

    if explicit_a:
        explicit_a_real = Path(explicit_a).resolve()
        remaining = [p for p in ports if Path(p.device).resolve() != explicit_a_real]
        if len(remaining) == 1:
            return explicit_a, remaining[0].device
        return explicit_a, prompt_choice(remaining or ports, "B")

    if len(ports) == 2:
        return ports[0].device, ports[1].device

    if len(ports) < 2:
        print(f"Only found {len(ports)} serial port(s): {[p.device for p in ports]}")
        print(
            "Need two connected radios. Plug in both devices and retry, pass --port-a/--port-b "
            "explicitly, or pass --single to test with just one radio."
        )
        sys.exit(1)

    print(f"Found {len(ports)} serial ports, ambiguous which two are the radios.")
    a = prompt_choice(ports, "A")
    remaining = [p for p in ports if p.device != a]
    b = prompt_choice(remaining, "B")
    return a, b


def main():
    sys.stdout.reconfigure(line_buffering=True)  # otherwise stdout fully buffers when piped (e.g. into tee)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port-a", help="Serial device for radio A (skip auto-detect)")
    parser.add_argument("--port-b", help="Serial device for radio B (skip auto-detect)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--log-dir", default=".", help="Directory to write per-radio JSONL logs into (default: cwd)"
    )
    parser.add_argument("--no-file-log", action="store_true", help="Only print to console, don't write log files")
    parser.add_argument(
        "--single", action="store_true", help="Run with only one radio connected (for testing)"
    )
    parser.add_argument(
        "--rssi-monitor",
        action="store_true",
        help="Skip the packet-compare loop; configure radio A and print its live raw RSSI reading instead "
        "(implies --single). Use this to check whether the RF front-end detects any energy at all.",
    )
    parser.add_argument(
        "--summary-interval",
        type=float,
        default=DEFAULT_SUMMARY_INTERVAL_S,
        help=f"Seconds between printed stats summaries, 0 to disable (default: {DEFAULT_SUMMARY_INTERVAL_S}). "
        "A final summary is always printed on exit.",
    )
    parser.add_argument(
        "--rolling-window",
        type=int,
        default=DEFAULT_ROLLING_WINDOW,
        help=f"Number of most-recent packets used for rolling RSSI/SNR averages (default: {DEFAULT_ROLLING_WINDOW})",
    )
    parser.add_argument(
        "--match-window",
        type=float,
        default=DEFAULT_MATCH_WINDOW_S,
        help="Seconds to wait for the other radio to also report a packet before counting it as "
        f"'heard by only one radio' (default: {DEFAULT_MATCH_WINDOW_S})",
    )
    args = parser.parse_args()
    if args.rssi_monitor:
        args.single = True

    port_a, port_b = select_ports(args.port_a, args.port_b, single=args.single)
    print(f"Radio A: {port_a}")
    if port_b:
        print(f"Radio B: {port_b}")
    else:
        print("Radio B: (none — single-radio mode)")

    ser_a = serial.Serial(port_a, args.baud, timeout=READ_TIMEOUT_S)
    ser_b = serial.Serial(port_b, args.baud, timeout=READ_TIMEOUT_S) if port_b else None

    log_fh_a = log_fh_b = None
    if not args.no_file_log:
        ts = datetime.now().strftime("%Y%m%dT%H%M%S")
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_fh_a = open(log_dir / f"kiss_rx_{ts}_A.jsonl", "w")
        if port_b:
            log_fh_b = open(log_dir / f"kiss_rx_{ts}_B.jsonl", "w")

    link_a = RadioLink(label="A", port=port_a, ser=ser_a, log_fh=log_fh_a)
    link_b = RadioLink(label="B", port=port_b, ser=ser_b, log_fh=log_fh_b) if port_b else None
    links = [link_a] + ([link_b] if link_b else [])
    for link in links:
        link.rolling_rssi = deque(maxlen=args.rolling_window)
        link.rolling_snr = deque(maxlen=args.rolling_window)

    node_table = NodeTable()
    matcher = (
        CrossRadioMatcher(window_s=args.match_window, rolling_window=args.rolling_window) if link_b else None
    )

    print_lock = threading.Lock()

    print(f"Configuring US-band defaults on {'both radios' if link_b else 'the radio'}...")
    for link in links:
        configure_us_defaults(link, print_lock)
        run_radio_diagnostics(link, print_lock)

    if args.rssi_monitor:
        rssi_monitor(link_a)
        link_a.ser.close()
        return

    stop_event = threading.Event()
    start_time = time.time()
    threads = (
        [
            threading.Thread(
                target=reader_thread, args=(link, print_lock, stop_event, matcher, node_table), daemon=True
            )
            for link in links
        ]
        + [threading.Thread(target=stats_poll_thread, args=(link, stop_event), daemon=True) for link in links]
        + [
            threading.Thread(
                target=housekeeping_thread,
                args=(links, matcher, node_table, stop_event, print_lock, args.summary_interval, start_time),
                daemon=True,
            )
        ]
    )
    for t in threads:
        t.start()

    sigint_count = 0

    def handle_sigint(signum, frame):
        nonlocal sigint_count
        sigint_count += 1
        if sigint_count == 1:
            stop_event.set()
        else:
            os._exit(1)

    signal.signal(signal.SIGINT, handle_sigint)

    print(f"\nListening for packets on {'both radios' if link_b else 'the radio'}. Press Ctrl+C to stop.\n")
    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=1.0)
        if matcher is not None:
            emit_unique_events({link.label: link for link in links}, matcher.flush_all(), print_lock)
        for link in links:
            link.ser.close()
            if link.log_fh:
                link.log_fh.close()
        print(render_summary(links, matcher, node_table, start_time))
        print("\nDone.")


if __name__ == "__main__":
    main()
