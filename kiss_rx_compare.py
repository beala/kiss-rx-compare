#!/usr/bin/env python3
"""
Compare LoRa reception between two MeshCore KISS-modem radios.

Connects to both serial devices, configures each for US-band defaults
(915 MHz / 250 kHz BW / SF10 / CR5 / 20 dBm), and logs every received
packet from both radios side by side (payload + RSSI/SNR/timing) so
their reception can be compared.

Requires: pyserial (pip install pyserial)
"""

import argparse
import json
import struct
import sys
import threading
import time
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
HW_CMD_GET_VERSION = 0x11
HW_CMD_GET_DEVICE_NAME = 0x16
HW_CMD_PING = 0x17

HW_RESP_OK = 0xF0
HW_RESP_ERROR = 0xF1
HW_RESP_TX_DONE = 0xF8
HW_RESP_RX_META = 0xF9


def hw_resp(cmd):
    return cmd | 0x80


# US-band defaults, matching examples/companion_radio/MyMesh.h LORA_* defaults.
US_FREQ_HZ = 915_000_000
US_BW_HZ = 250_000
US_SF = 10
US_CR = 5
US_TX_POWER_DBM = 20

READ_TIMEOUT_S = 0.2
STARTUP_RESPONSE_TIMEOUT_S = 2.0


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


@dataclass
class RadioLink:
    label: str
    port: str
    ser: serial.Serial
    decoder: KissDecoder = field(default_factory=KissDecoder)
    pending: "PendingPacket | None" = None
    packet_count: int = 0
    log_fh: object = None


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


def format_console(event: dict) -> str:
    kind = event["type"]
    prefix = f"[{event['time']}] {event['radio']:>1} "
    if kind == "packet":
        return (
            f"{prefix}RX len={event['len']:<3} "
            f"rssi={event['rssi_dbm']:>4} dBm snr={event['snr_db']:>6.2f} dB  "
            f"payload={event['payload_hex']}"
        )
    if kind == "meta_only":
        return f"{prefix}META (no matching packet) rssi={event['rssi_dbm']} snr={event['snr_db']:.2f}"
    if kind == "packet_no_meta":
        return f"{prefix}RX len={event['len']:<3} (no meta received) payload={event['payload_hex']}"
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


def handle_frame(link: RadioLink, frame: bytes, print_lock: threading.Lock):
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
                    },
                    print_lock,
                )
            else:
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

        log_event(link, {"type": "hw_resp", "sub_cmd": sub_cmd, "payload_hex": hexdump(payload)}, print_lock)


def reader_thread(link: RadioLink, print_lock: threading.Lock, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            data = link.ser.read(4096)
        except serial.SerialException as e:
            log_event(link, {"type": "info", "message": f"serial error: {e}"}, print_lock)
            return
        if not data:
            continue
        for frame in link.decoder.feed(data):
            handle_frame(link, frame, print_lock)


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


def configure_us_defaults(link: RadioLink, print_lock: threading.Lock):
    radio_payload = struct.pack("<IIBB", US_FREQ_HZ, US_BW_HZ, US_SF, US_CR)
    link.ser.write(encode_hw_frame(HW_CMD_SET_RADIO, radio_payload))
    ok, err = wait_for_hw_response(link)
    log_event(
        link,
        {
            "type": "info",
            "message": f"SET_RADIO({US_FREQ_HZ / 1e6:.1f} MHz, {US_BW_HZ / 1e3:.0f} kHz, SF{US_SF}, CR{US_CR}) "
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

def discover_ports():
    return sorted(list_ports.comports(), key=lambda p: p.device)


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


def select_ports(explicit_a, explicit_b):
    if explicit_a and explicit_b:
        return explicit_a, explicit_b

    ports = discover_ports()
    if explicit_a:
        remaining = [p for p in ports if p.device != explicit_a]
        if len(remaining) == 1:
            return explicit_a, remaining[0].device
        return explicit_a, prompt_choice(remaining or ports, "B")

    if len(ports) == 2:
        return ports[0].device, ports[1].device

    if len(ports) < 2:
        print(f"Only found {len(ports)} serial port(s): {[p.device for p in ports]}")
        print("Need two connected radios. Plug in both devices and retry, or pass --port-a/--port-b explicitly.")
        sys.exit(1)

    print(f"Found {len(ports)} serial ports, ambiguous which two are the radios.")
    a = prompt_choice(ports, "A")
    remaining = [p for p in ports if p.device != a]
    b = prompt_choice(remaining, "B")
    return a, b


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port-a", help="Serial device for radio A (skip auto-detect)")
    parser.add_argument("--port-b", help="Serial device for radio B (skip auto-detect)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--log-dir", default=".", help="Directory to write per-radio JSONL logs into (default: cwd)"
    )
    parser.add_argument("--no-file-log", action="store_true", help="Only print to console, don't write log files")
    args = parser.parse_args()

    port_a, port_b = select_ports(args.port_a, args.port_b)
    print(f"Radio A: {port_a}")
    print(f"Radio B: {port_b}")

    ser_a = serial.Serial(port_a, args.baud, timeout=READ_TIMEOUT_S)
    ser_b = serial.Serial(port_b, args.baud, timeout=READ_TIMEOUT_S)

    log_fh_a = log_fh_b = None
    if not args.no_file_log:
        ts = datetime.now().strftime("%Y%m%dT%H%M%S")
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_fh_a = open(log_dir / f"kiss_rx_{ts}_A.jsonl", "w")
        log_fh_b = open(log_dir / f"kiss_rx_{ts}_B.jsonl", "w")

    link_a = RadioLink(label="A", port=port_a, ser=ser_a, log_fh=log_fh_a)
    link_b = RadioLink(label="B", port=port_b, ser=ser_b, log_fh=log_fh_b)

    print_lock = threading.Lock()

    print("Configuring US-band defaults on both radios...")
    configure_us_defaults(link_a, print_lock)
    configure_us_defaults(link_b, print_lock)

    stop_event = threading.Event()
    threads = [
        threading.Thread(target=reader_thread, args=(link_a, print_lock, stop_event), daemon=True),
        threading.Thread(target=reader_thread, args=(link_b, print_lock, stop_event), daemon=True),
    ]
    for t in threads:
        t.start()

    print("\nListening for packets on both radios. Press Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=1.0)
        ser_a.close()
        ser_b.close()
        if log_fh_a:
            log_fh_a.close()
        if log_fh_b:
            log_fh_b.close()
        print(f"\nDone. Radio A received {link_a.packet_count} packet(s), Radio B received {link_b.packet_count} packet(s).")


if __name__ == "__main__":
    main()
