# SPACEBOT WFB-ng FPV Streaming — Session Summary
> Dibuat untuk konteks AI sesi berikutnya. Baca seluruh dokumen ini sebelum menjawab.

---

## 1. Identitas User

- **Nama**: Muhammad Zidan Apriadi (biasa dipanggil Zidan)
- **NIM**: 22501244017, Pendidikan Teknik Elektro UNY
- **Project**: SPACEBOT — FPV rover dengan AI vision + WFB-ng long-range video

---

## 2. Stack Hardware

```
[K230 HuskyLens 2] --TCP:8554 H264--> [RPi 4 Air Unit] --WFB-ng RF--> [GCS Linux]
       AI encoder                          AR9271 TX                   AR9271 RX
  192.168.88.1 (RNDIS)               wlan0, /etc/drone.key       wlxf4ec3889c21c
                                                                  gs.key di ~/wfb-ng/
```

| Komponen | Detail |
|---|---|
| K230 SoC | Kendryte K230 RISC-V, HuskyLens 2 |
| K230 encoder | Linlon h264_v4l2m2m (hardware) |
| K230 OS | Custom Linux, kernel 6.6.36 riscv64 |
| K230 IP | 192.168.88.1 via USB RNDIS |
| RPi air unit | Raspberry Pi 4, hostname `robot`, user `spacebot21` |
| WFB-ng adapter | AR9271 (TP-Link TL-WN722N V1) |
| WFB-ng channel | 7 |
| WFB-ng port | 5600 |
| WFB-ng keys | TX: `/etc/drone.key` | RX: `$HOME/wfb-ng/gs.key` (matched pair) |
| GCS program | `debug.py` di `~/esp32_alarm/` (PyQt5 ~1800 lines) |

---

## 3. BUG PERMANEN K230 Encoder — WAJIB DIPAHAMI

Linlon h264_v4l2m2m di K230 punya **bug hardware** yang tidak bisa diperbaiki tanpa ganti firmware:

```
Error: [h264] sps_id 1 out of range
Error: [h264] non-existing PPS 1 referenced
```

**Penjelasan**: Encoder secara permanen output slice yang mereferensikan PPS ID=1, tapi tidak pernah transmit PPS NAL dengan ID=1. Error ini muncul di **semua** upaya decode.

**Dampak**:
- ffmpeg dengan `-c:v copy` (TX remux): TIDAK bermasalah — tidak perlu decode
- ffplay/ffmpeg decode langsung: Bisa ditoleransi dengan flag `-err_detect ignore_err`
- MPEG-TS decoder strict: BLOKIR TOTAL — tidak bisa display sama sekali

**Solusi yang sudah terbukti**: Gunakan raw H.264 end-to-end, bukan MPEG-TS untuk display.

---

## 4. Pipeline yang SUDAH CONFIRMED WORKING

### 4.1 Alur lengkap

```
K230 spacebot_ai.py
  └── ffmpeg rawvideo → h264_v4l2m2m → TCP:8554 (raw H.264)
        |
        | TCP connect
        v
RPi tx_video.sh
  ├── wfb_tx -p 0 -u 5600 -K /etc/drone.key wlan0
  └── ffmpeg -c:v copy -f mpegts → UDP:5600 → wfb_tx
        |
        | WFB-ng wireless (channel 7)
        v
GCS rx_video.sh
  └── wfb_rx -p 0 -u 5600 -K ~/wfb-ng/gs.key wlxf4ec3889c21c
        |
        | UDP:127.0.0.1:5600
        v
GCS debug.py (VideoThread)
  └── ffmpeg udp://127.0.0.1:5600 → rawvideo → PyQt5 display
```

### 4.2 Cara jalankan (urutan wajib)

```bash
# Terminal 1 — GCS machine
./rx_video.sh            # jalankan wfb_rx, TANPA ffplay

# Terminal 2 — GCS machine
cd ~/esp32_alarm
python3 debug.py         # login: 1234
# klik START STREAM di panel VIDEO FEED
```

---

## 5. Script Konfigurasi Final

### 5.1 K230: `/root/spacebot_ai.py` — fungsi `start_ffmpeg()`

```python
def start_ffmpeg():
    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", "%dx%d" % (CAM_W, CAM_H),
        "-r", str(STREAM_FPS),
        "-i", "-",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-pix_fmt", "yuv420p",
        "-c:v", "h264_v4l2m2m",
        "-b:v", "300k",          # JANGAN kurang dari 250k (lihat section 7)
        "-maxrate", "300k",      # cap bitrate, cegah keyframe spike
        "-bufsize", "300k",      # VBV buffer ketat = CBR konsisten
        "-g", "10",              # keyframe setiap 10 frame (bukan 5)
        "-bf", "0",              # no B-frame = low latency
        "-sc_threshold", "0",   # matikan scene change detection
        "-f", "h264",
        "tcp://0.0.0.0:%d?listen=1&tcp_nodelay=1" % STREAM_PORT
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=0)
```

**Parameter K230 lainnya:**
```python
KMODEL_PATH    = "/root/yolo11n.kmodel"
INPUT_SIZE     = 224
CAM_W, CAM_H   = 640, 480
STREAM_FPS     = 25
STREAM_PORT    = 8554
TELEMETRY_PORT = 5555
```

### 5.2 RPi TX: `tx_video.sh`

```bash
#!/bin/bash
WLAN="wlan0"
KEY="/etc/drone.key"
CHANNEL=7
PORT=5600
K230_IP="192.168.88.1"
STREAM_PORT="8554"

# Set monitor mode
sudo ip link set $WLAN down
sudo iw dev $WLAN set type monitor
sudo ip link set $WLAN up
sudo iw dev $WLAN set channel $CHANNEL

# Start wfb_tx — TANPA FEC flags (-k/-n), TANPA MCS flags (-M)
sudo wfb_tx -p 0 -u $PORT -K $KEY $WLAN &
sleep 1

# Forward K230 TCP -> WFB-ng UDP
ffmpeg \
    -fflags nobuffer \
    -flags low_delay \
    -probesize 32 \
    -analyzeduration 0 \
    -max_delay 0 \
    -avioflags direct \
    -i tcp://$K230_IP:$STREAM_PORT \
    -an \
    -c:v copy \
    -f mpegts \
    -flush_packets 1 \
    -max_muxing_queue_size 64 \
    "udp://127.0.0.1:$PORT?pkt_size=1316"
```

### 5.3 GCS RX: `rx_video.sh` (TANPA ffplay — GCS yang handle display)

```bash
#!/bin/bash
WLAN="wlxf4ec3889c21c"
KEY="$HOME/wfb-ng/gs.key"
CHANNEL=7
PORT=5600

cleanup() {
    sudo pkill -f wfb_rx
    exit 0
}
trap cleanup SIGINT SIGTERM

sudo pkill -f wfb_rx

sudo ip link set $WLAN down
sudo iw dev $WLAN set type monitor
sudo ip link set $WLAN up
sudo iw dev $WLAN set channel $CHANNEL

# Start wfb_rx — TANPA FEC flags, harus match TX
sudo wfb_rx -p 0 -u $PORT -K $KEY $WLAN
cleanup
```

### 5.4 GCS VideoThread (`debug.py`) — ffmpeg flags

```python
# VideoThread.run() — sudah di-patch dengan patch_gcs.py
stream_url = "udp://127.0.0.1:5600?overrun_nonfatal=1"

cmd = [
    'ffmpeg',
    '-fflags',          'nobuffer+discardcorrupt',
    '-flags',           'low_delay',
    '-probesize',       '32',
    '-analyzeduration', '0',
    '-max_delay',       '0',
    '-avioflags',       'direct',
    '-err_detect',      'ignore_err',
    '-i', stream_url,
    '-vf', f'scale={width}:{height}',
    '-f', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-an', '-sn', '-'
]
```

---

## 6. ATURAN KRITIS — JANGAN DILANGGAR

### 6.1 wfb_tx dan wfb_rx HARUS simetris
```bash
# BENAR
sudo wfb_tx -p 0 -u 5600 -K /etc/drone.key wlan0
sudo wfb_rx -p 0 -u 5600 -K ~/wfb-ng/gs.key wlxf4ec3889c21c

# SALAH — FEC mismatch = RX tidak terima apapun
sudo wfb_tx -p 0 -u 5600 -K key -k 4 -n 6 wlan0   # FEC di TX
sudo wfb_rx -p 0 -u 5600 -K key wlan0              # tidak di RX
```

### 6.2 Bash line continuation
```bash
# BENAR
ffmpeg \
    -fflags nobuffer \
    -i udp://...

# SALAH — komentar setelah backslash merusak line continuation
ffmpeg \
    -fflags nobuffer \ # ini mematikan streaming
    -i udp://...
```

### 6.3 Port binding
- wfb_rx output ke UDP:5600
- Hanya **satu proses** yang bisa bind port 5600 sekaligus
- Kalau `rx_video.sh` pakai ffplay DAN debug.py juga nyambung → salah satu gagal
- Solusi: `rx_video.sh` TANPA ffplay, debug.py yang handle display

---

## 7. Bitrate K230 — Hasil Testing

| Bitrate | Hasil |
|---|---|
| `1200k` | Patah-patah, terlalu berat |
| `300k` | **OPTIMAL** — smooth, low latency |
| `250k` | Aman |
| `200k` | **PATAH-PATAH** meski lebih rendah dari 300k |
| `100k` | Lancar tapi kualitas buruk |

**Kenapa 200k lebih buruk dari 300k:**
Dengan `-g 5` (keyframe setiap 5 frame), keyframe (I-frame) jauh lebih besar dari P-frame. Di 200k, keyframe spike meledak budget → pipeline stall → patah-patah. Di 300k ada cukup headroom. Solusi selain naikkan bitrate: naikkan `-g` (GOP size) agar keyframe lebih jarang, atau tambahkan `-maxrate`/`-bufsize`.

---

## 8. Overclocking RPi 4 — Tidak Perlu

Task RPi 4 di TX sangat ringan:
- `ffmpeg -c:v copy` = hanya remux container, hampir 0% CPU
- `wfb_tx` = I/O bound bukan CPU bound
- Di 300k stream, RPi 4 idle ~5-10% CPU

Overclocking berisiko:
- RPi 4 sudah panas, OC memperparah → thermal throttle → lebih lambat
- Power naik → baterai rover lebih cepat habis
- Instabilitas bisa crash wfb_tx

Cek throttle:
```bash
vcgencmd get_throttled
# 0x0 = normal, 0x50005 = throttle aktif/pernah terjadi
```

---

## 9. File & Lokasi Penting

| File | Lokasi | Keterangan |
|---|---|---|
| K230 main script | `/root/spacebot_ai.py` | AI + streaming, JANGAN hapus |
| K230 backup | `/root/spacebot_rover_final_BACKUP_19700101.py` | Backup aman |
| K230 kmodel | `/root/yolo11n.kmodel` | YOLO11n, INPUT_SIZE=224 |
| RPi TX script | `~/tx_video.sh` (di RPi) | WFB-ng TX |
| GCS program | `~/esp32_alarm/debug.py` | PyQt5 GCS |
| GCS backup | `~/esp32_alarm/debug.py.bak` | Backup sebelum patch |
| GCS RX script | `~/rx_video.sh` (di GCS) | wfb_rx tanpa ffplay |
| Patch script | `~/esp32_alarm/patch_gcs.py` | Sudah diapply, tidak perlu lagi |

---

## 10. Riwayat Debugging Penting

### Problem 1: Video tidak muncul sama sekali (awal)
- **Sebab**: `probesize 32` terlalu kecil, SPS K230 tidak terdeteksi
- **Fix**: Sudah di-handle oleh `-err_detect ignore_err` + raw H264 pipeline

### Problem 2: MPEG-TS output K230 rusak
- **Sebab**: `-muxdelay 0 -muxpreload 0` merusak PCR timestamp di MPEG-TS
- **Fix**: Hapus parameter itu. K230 output raw H264 (`-f h264`), bukan MPEG-TS

### Problem 3: TX error "dimensions not set"
- **Sebab**: `-f h264` output di TX butuh dimensi eksplisit; tanpa decode, dimensi tidak diketahui
- **Fix**: TX output ke `-f mpegts` bukan `-f h264` (MPEG-TS tidak butuh dimensi eksplisit)

### Problem 4: RX tidak terima data padahal TX jalan
- **Sebab**: FEC mismatch — TX pakai `-k 4 -n 6`, RX tidak pakai flag FEC
- **Fix**: Hapus semua FEC flag dari kedua sisi. Default wfb_tx/wfb_rx sudah simetris

### Problem 5: Video patah-patah di bitrate rendah (200k)
- **Sebab**: Keyframe spike meledak budget VBV di bitrate rendah
- **Fix**: Gunakan 300k + tambah `-maxrate 300k -bufsize 300k -g 10`

### Problem 6: Port conflict ffplay vs debug.py
- **Sebab**: Keduanya mencoba bind UDP:5600
- **Fix**: rx_video.sh hanya jalankan wfb_rx (tanpa ffplay), debug.py handle display

---

## 11. Status Akhir Sesi

| Komponen | Status |
|---|---|
| K230 H264 encoding | Jalan, bug PPS permanen tapi sudah di-handle |
| WFB-ng TX link | Jalan, ~107-200 pkts/sec confirmed |
| WFB-ng RX link | Jalan, realtime confirmed (alhamdulillah) |
| GCS debug.py VideoThread | Sudah di-patch ke port 5600 |
| rx_video.sh | Sudah diupdate tanpa ffplay |
| Bitrate optimal | 300k dengan `-maxrate -bufsize -g 10 -sc_threshold 0` |

---

## 12. Todo / Yang Belum Diselesaikan

- [ ] Apply optimized K230 ffmpeg command (dengan `-maxrate`, `-bufsize`, `-g 10`, `-sc_threshold 0`, `tcp_nodelay=1`) ke `/root/spacebot_ai.py`
- [ ] Investigasi second USB path di HuskyLens 2 PCB (`usb@91540000`, `dr_mode=host`) untuk USB webcam eksternal
- [ ] Custom OS injection plan: RNDIS service, RTSP GStreamer, nncase kmodel, mosquitto MQTT, detection_trigger.py

---

*Dokumen ini dibuat dari rangkuman sesi debugging WFB-ng + K230 FPV streaming pada 2026-06-06.*
