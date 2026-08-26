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

## Live split-screen view

When stdout is a terminal, the listening phase defaults to a live two-pane
view: a scrolling packet log on the left, a continuously-updated stats
summary on the right (press `q` or Ctrl+C to quit). Pass `--ui plain` for
the original single-stream console (used automatically when stdout isn't a
terminal, e.g. piped into `tee`), or `--ui split`/`--ui auto` to control it
explicitly. The JSONL log files are written the same way regardless of
which UI is active.

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

### Packets vs. payloads

A flood-routed packet gets relayed by every repeater that hears it, and
each relay is a distinct over-the-air reception (different path, different
timing, different RSSI/SNR) even though it's the same underlying message.
The summary tracks both:

- **Packets** — every individual reception, exactly as it arrived.
- **Payloads** — messages deduplicated the same way MeshCore's own
  flood-routing dedup does it (payload type + payload bytes, ignoring the
  path/header that changes at each hop), so ten repeater copies of the same
  advert count once.

Per radio, this shows up as `packets: N  unique payloads: M (repeated via
relay: N-M)`. Across radios, "heard by only one radio" is reported at both
levels: the packet-level match (`--match-window`) only looks at a short
window and can flag two receptions as mismatched just because a relay copy
arrived outside it, even though both radios did eventually get the
message. The payload-level tally (session-lifetime, no window) is the more
meaningful one for spotting an actual reception gap — a message one radio
never received in any form — and should generally be small.
