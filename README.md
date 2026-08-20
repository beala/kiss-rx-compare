# kiss-rx-compare

Compares LoRa reception between two [MeshCore](https://github.com/meshcore-dev/MeshCore)
radios running the `kiss_modem` firmware example.

Connects to both serial devices, configures each for US-band defaults
(915 MHz / 250 kHz BW / SF10 / CR5 / 20 dBm), and logs every received
packet from both radios side by side (payload + RSSI/SNR) so their
reception can be compared.

## Usage

```bash
pip install -r requirements.txt
python3 kiss_rx_compare.py
```

The two radios should be the only serial devices connected to the
machine. If port auto-detection is ambiguous, the script prompts for
which port is which. Use `--port-a`/`--port-b` to specify explicitly.

Logs are printed to the console and written as per-radio JSONL files
(`kiss_rx_<timestamp>_A.jsonl` / `_B.jsonl`) in `--log-dir` (default:
cwd). Pass `--no-file-log` to only print to console.

```
python3 kiss_rx_compare.py --port-a /dev/ttyACM0 --port-b /dev/ttyACM1 --log-dir ./logs
```
