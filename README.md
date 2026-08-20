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
