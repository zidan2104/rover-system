# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

This is the **SPACEBOT** FPV rover project — an AI-vision rover with long-range WFB-ng
(WiFi broadcast) video. The repo aggregates source for code that runs on **three different
machines**; editing a file here does nothing until it is deployed to its target device:

| File | Runs on | Role |
|---|---|---|
| `debug.py` | GCS laptop (Linux Mint/Ubuntu) | PyQt5 Ground Control Station (~3300 lines, single file) |
| `spacebot_ai.py` | K230 (Kendryte RISC-V AI SoC) → `/root/spacebot_ai.py` | YOLO11n inference + H264 HW encode + stream |
| `tx_video.sh` | RPi 4 air unit | Bridges K230 TCP stream → WFB-ng RF |
| `rx_video.sh` | GCS laptop | Runs `wfb_rx` only (display is done by `debug.py`) |

`SPACEBOT_WFBng_Session_Summary.md` is the **canonical reference** for hardware wiring,
WFB-ng key/channel setup, the K230 encoder PPS bug, bitrate test results, and debugging
history. Read it before changing anything in the video pipeline.

## Running and developing

There is no build system, test suite, or linter. Python files are validated with
`python -m py_compile`. The GCS app **must** run inside its virtualenv (python-mpv and
deps live there).

```bash
# GCS — two terminals, both with the venv active
cd ~/esp32_alarm && source venv/bin/activate
./rx_video.sh                 # terminal 1: wfb_rx -> UDP 127.0.0.1:5600
python3 debug.py              # terminal 2: GCS GUI; login passcode = 1234, then click START STREAM

python -m py_compile debug.py # syntax check after edits
python3 test_mpv.py                         # isolate mpv embedding (no stream)
python3 test_mpv.py udp://127.0.0.1:5600    # isolate mpv with the live stream
python3 debug.py --terminal   # run without GUI (skips QApplication)
```

Deploying device code: copy `spacebot_ai.py` to the K230 and restart its AI program;
copy `tx_video.sh` to the RPi and re-run `./tx_video.sh`. `LOCKED_20260608/` holds the
last known-good snapshot of all four files; `*.bak` files are point-in-time backups.

## Two independent data planes

The GCS receives video and telemetry over **completely separate paths** — they are not
correlated in software:

1. **Control + robot telemetry** — an ESP32 ground controller connects to the GCS over
   USB serial @115200. `SerialHandler` reads lines in a background thread and
   `SpacebotData.parse()` decodes a fixed 28-field ASCII protocol
   (`$DATA|SWSTR|J1X|...|HDG`, documented in the header comment of `debug.py`). Outbound
   commands use the same channel, e.g. servo config `$SCFG|ch|trim|epaL|epaR`.

2. **Video** — `K230 camera → YOLO boxes burned into frame → H264 (h264_v4l2m2m) →
   TCP:8554 → RPi ffmpeg -c:v copy → mpegts UDP:5600 → wfb_tx → RF → wfb_rx →
   udp://127.0.0.1:5600 → debug.py (libmpv)`. Detection boxes are rendered **on the K230**
   and encoded into the video. The K230 also exposes JSON telemetry on TCP:5555, but
   `debug.py` does **not** consume it today (a future "decouple AI" refactor would stream
   raw video and draw boxes GCS-side from 5555 to cut latency).

## `debug.py` structure (single-file PyQt5)

Everything is gated by `GUI_AVAILABLE` / `OPENCV_AVAILABLE` / `MPV_AVAILABLE` flags so the
file imports even when a dependency is missing. Key classes: `SpacebotData` (telemetry
parser), `SerialHandler`/`SerialSignals` (serial thread), `VideoStreamingWidget` (video),
`OfflineMapWidget` (OSM tiles from `map_tiles/`, trail + home marker),
`JoystickWidget`/`AttitudeWidget`/`CompassWidget`/`BigValueWidget` (HUD),
`AlarmSystem` (tone generation via `aplay` on Linux / `winsound` on Windows), and
`SpacebotGCS(QMainWindow)` whose `update_timer` (50 ms) drives `update_display()`.
`gcs_config.json` persists alarm thresholds and per-channel servo trim/EPA
(`SERVO_NUM_CH = 9`).

## Video pipeline — non-obvious constraints (these caused real, repeated failures)

Video is rendered by **embedding libmpv** into a Qt native window
(`VideoStreamingWidget._ensure_mpv`). The following are hard requirements, not preferences:

- **Run inside the venv.** `python-mpv` is installed there; outside it `MPV_AVAILABLE` is
  False and video silently won't appear.
- **libmpv needs `LC_NUMERIC="C"`** or it segfaults on init. `QApplication` resets the
  locale to the system locale (Indonesian uses a comma decimal), so `setlocale` is called
  again in `main()` and once more immediately before `mpv.MPV(...)`. Do not remove either.
- **Create mpv lazily**, after the window is shown (in `start_stream`, not `__init__`).
  Creating it before the native window's `winId()` is realized crashes the app.
- **Distro package is `libmpv1`** on Ubuntu 22.04 / Mint 21 (`libmpv2` does not exist there).
- **K230 `h264_v4l2m2m` has a permanent PPS bug** (`non-existing PPS 1 referenced`). Decode
  must tolerate it: software decode (`hwdec=no`) + `demuxer-lavf-o=fflags=+nobuffer+discardcorrupt`.
  A strict MPEG-TS decode path blocks video entirely.
- **`wfb_tx` and `wfb_rx` must be symmetric** (same `-p`, `-u`, key; no FEC-flag mismatch)
  or the RX receives nothing.
- **Only one process may bind UDP:5600.** `rx_video.sh` runs `wfb_rx` only; `debug.py` owns
  the display. Never also start `ffplay` in `rx_video.sh`.

Low-latency tuning lives in `_ensure_mpv` (`profile=low-latency`, `untimed`, `cache=no`,
`framedrop=vo`, `panscan=1.0` to remove pillarbox) and in the device scripts
(`tcp_nodelay=1`, `-flush_packets 1`, `-max_delay 0`, `vpu_queue maxsize=1`). `vo=gpu` keeps
vsync on for smoothness; disabling vsync was tried and rejected (tearing). Liveness is
detected via the mpv `core-idle` property because `estimated-vf-fps` reads 0 under
`untimed`.
