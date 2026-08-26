# kiss-rx-compare

Compares LoRa reception between two [MeshCore](https://github.com/meshcore-dev/MeshCore)
radios running the `kiss_modem` firmware example.

Connects to both serial devices, configures each for the MeshCore
USA/Canada preset (910.525 MHz / 62.5 kHz BW / SF7 / CR5, TX power
set to 0 dBm for close-range bench testing), and logs every received
packet from both radios side by side (payload + RSSI/SNR) so their
reception can be compared.

## Usage

This is a [uv](https://docs.astral.sh/uv/) project. `uv run` installs
dependencies into a local `.venv` automatically on first use.

```bash
uv run kiss_rx_compare.py
```

The two radios should be the only serial devices connected to the
machine. If port auto-detection is ambiguous, the script prompts for
which port is which. Use `--port-a`/`--port-b` to specify explicitly
(a `/dev/serial/by-id/...` path is preferred automatically when one
exists for the detected device).

Logs are printed to the console and written as per-radio JSONL files
(`kiss_rx_<timestamp>_A.jsonl` / `_B.jsonl`) in `--log-dir` (default:
cwd). Pass `--no-file-log` to only print to console.

```bash
uv run kiss_rx_compare.py --port-a /dev/ttyACM0 --port-b /dev/ttyACM1 --log-dir ./logs
```

Pass `--single` to run with only one radio connected, for testing:

```bash
uv run kiss_rx_compare.py --single
```

## Packet decoding & comparison stats

Each received packet is decoded (best-effort, cleartext fields only — no
keys are involved) using MeshCore's on-air packet format, so console/log
lines show things like `type=ADVERT name="My Node"` or `type=TXT_MSG
dest=ab src=cd` instead of just raw hex. The decoded fields are stored
under an `"mc"` key in each `packet` JSONL record.

When both radios are connected, packets are correlated across radios by
matching identical payload bytes within a short window (`--match-window`,
default 3s) — two nearby radios receiving the same over-the-air
transmission decode identical bytes, so this reliably tells "both radios
heard this" apart from "only one radio heard this" without needing to
decrypt anything.

A periodic stats summary (`--summary-interval`, default 30s, 0 to
disable) and a final summary on exit report, per radio: packet counts,
cumulative decode/CRC errors (from polling `GetStats`), payload-type
breakdown, and all-time + rolling-window (`--rolling-window`, default 20
packets) RSSI/SNR averages. When both radios are present it also reports
packets heard by only one radio (split out by which), and the RSSI/SNR
delta between radios for packets both heard. Distinct nodes seen via
ADVERT packets (name, type, per-radio hit counts and last signal) are
listed too, since MeshCore adverts are sent unencrypted and self-signed.

The JSONL log itself is unaffected by summaries — it stays a full,
unaggregated event stream; `"unique_to_radio"` events are added to a
radio's log when the match window expires without the other radio
reporting the same packet.
