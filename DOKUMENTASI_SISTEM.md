# SPACEBOT — Dokumentasi Sistem

**FPV Rover berbasis AI-vision dengan video long-range WFB-ng.**
Penjelasan menyeluruh cara kerja seluruh program & sistemnya — dari kamera di rover
sampai tampil di layar operator.

> Author proyek: **Zidan (Muhammad Zidan Apriadi)** — SPACEBOT Project
> Dokumen pendamping:
> - `CLAUDE.md` — panduan singkat untuk tooling/AI
> - `SPACEBOT_WFBng_Session_Summary.md` — referensi kanonik hardware, key WFB, bug encoder K230, hasil uji bitrate

---

## 1. Apa Ini & Tujuannya

SPACEBOT adalah **rover FPV** (First-Person View) yang:
1. Mengirim **video real-time low-latency** jarak jauh lewat **WFB-ng** (WiFi Broadcast — bukan WiFi biasa, tapi siaran RF satu arah tanpa asosiasi/handshake, tahan jarak jauh).
2. Menjalankan **AI vision di rover** (deteksi objek YOLO11n + **penguncian target / object tracking**), bukan di laptop. Kotak deteksi & HUD taktis **digambar langsung di K230** lalu ikut ter-encode ke video.
3. Dikendalikan dari **Ground Control Station (GCS)** — aplikasi desktop PyQt5 dengan video, peta offline, telemetry, dan tombol kontrol.

**Prinsip desain utama:** latency serendah mungkin + tahan link RF yang lossy.

---

## 2. Tiga Mesin (Siapa Menjalankan Apa)

Repo ini berisi kode untuk **3 perangkat berbeda**. Mengedit file di sini **tidak berefek** sampai di-*deploy* ke perangkat tujuannya.

| File | Jalan di | Peran |
|---|---|---|
| `debug.py` | **GCS** laptop (Linux Mint/Ubuntu) | Aplikasi Ground Control Station PyQt5 (~3300 baris, 1 file) |
| `rx_video.sh` | **GCS** laptop | Terima video RF + kirim perintah LOCK ke rover |
| `spacebot_ai.py` + `nanotrack_kpu.py` | **K230** (Kendryte RISC-V AI SoC) → `/root/` | YOLO11n + NanoTrack (full-KPU) + encode H264 + stream |
| `tx_video.sh` | **RPi 4** (air unit) | Jembatan: stream K230 → RF WFB-ng, dan relay perintah LOCK ke K230 |

```
   ┌─────────────────── ROVER (udara) ───────────────────┐        ┌──────── GCS (darat) ────────┐
   │                                                      │        │                             │
   │   K230 (AI SoC)            RPi 4 (air unit)          │  RF    │   Laptop GCS                │
   │   spacebot_ai.py   ──TCP── tx_video.sh   ───wfb_tx───┼──~~~~──┼──→ wfb_rx ──→ debug.py      │
   │   nanotrack_kpu.py  8554   (ffmpeg+wfb)   (-p0 video)│ ch.7   │   (rx_video.sh)  (libmpv)   │
   │        ▲                                  (-p1 lock)─┼──~~~~──┼──← wfb_tx ←── tombol LOCK    │
   │        │ USB RNDIS                                   │        │                             │
   │   192.168.88.1                                       │        │   USB serial 115200         │
   └─────────────────────────────────────────────────────┘        │        ▲                    │
                                                                   │   ESP32 controller ─────────┘
                                                                   │   (stik, switch, telemetry robot)
                                                                   └─────────────────────────────┘
```

---

## 3. Dua "Jalur Data" yang Terpisah Total

GCS menerima dua jenis data lewat **jalur yang benar-benar berbeda** dan **tidak dikorelasikan di software**:

### Jalur A — Kontrol + Telemetry Robot (kabel serial)
- **ESP32 ground controller** ↔ GCS lewat **USB serial @115200**.
- `SerialHandler` membaca baris di thread latar, `SpacebotData.parse()` mendekode protokol ASCII **28 field**:
  ```
  $DATA|SWSTR|J1X|J1Y|J1B|J2X|J2Y|J2B|NRF_TX|NRF_OK|NRF_CONN|NRF_Q|NRF_RSSI|LORA|R|P|Y|V|I|W|PRES|ALT|TEMP|LAT|LON|SPD|SAT|HDG
  ```
  - `SWSTR` = 7 digit: `SW1 SW2 SW3 SW4 SW5 BTN_MERAH BTN_HIJAU`
  - `R/P/Y` = roll/pitch/yaw, `V/I/W` = tegangan/arus/daya, `LAT/LON/SPD/SAT/HDG` = GPS, dst.
- **Perintah keluar** lewat channel yang sama, mis. konfigurasi servo: `$SCFG|ch|trim|epaL|epaR`.

### Jalur B — Video (RF WFB-ng)
- Kotak deteksi & HUD **dirender di K230**, lalu di-encode ke video. GCS hanya menampilkan.
- K230 juga membuka telemetry JSON di **TCP:5555**, tapi `debug.py` **belum** memakainya (rencana masa depan: "decouple AI" — stream video mentah lalu GCS gambar kotak dari 5555 untuk memangkas latency).

---

## 4. Pipeline Video (Detail Per Hop)

```
K230 kamera (640×480)
  └─ spacebot_ai.py: YOLO/NanoTrack burn kotak + HUD ke frame
       └─ ffmpeg h264_v4l2m2m (HW encode, 300k CBR, -g 10, -bf 0)
            └─ TCP :8554  (listen, tcp_nodelay=1)
   ─────────────────────────────────────── (USB RNDIS, 192.168.88.1)
RPi 4: tx_video.sh
  └─ ffmpeg -c:v copy (TANPA re-encode)  →  mpegts UDP :5600
       └─ wfb_tx -p0 -u5600  →  ►RF◄ channel 7
   ─────────────────────────────────────── (gelombang radio)
GCS: rx_video.sh
  └─ wfb_rx -p0 -u5600  →  UDP 127.0.0.1:5600
       └─ debug.py: libmpv (udp://127.0.0.1:5600)  →  layar
```

**Kenapa begini:**
- Encode H264 **di K230** (hardware `h264_v4l2m2m`) → ringan, kotak AI sudah menyatu di gambar.
- RPi cuma **`-c:v copy`** (salin tanpa encode ulang) → nyaris nol tambahan latency, sekadar membungkus ke MPEG-TS untuk WFB.
- WFB-ng = siaran RF (broadcast), bukan WiFi biasa → tak ada handshake/retransmit, cocok jarak jauh.
- libmpv (GPU + vsync) di GCS → tampilan **semulus ffplay**.

---

## 5. Fitur Inti: Deteksi → Lock → Tracking

Ini bagian "otak" rover (`spacebot_ai.py`). Loop utama berjalan **se-laju kamera** (proses tiap frame baru saja → anti patah-patah). Ada **dua mode**:

### Mode SCAN (default) — YOLO11n deteksi
- Resize frame ke **224×224** → inferensi **YOLO11n di KPU** → NMS → gambar semua kotak hijau + label kelas COCO.
- Selama **belum** ada perintah LOCK, tetap di SCAN.

### Mode LOCK/TRACK — NanoTrack (full-KPU)
Saat operator menekan LOCK (lihat §6), pada frame berikutnya:
1. Pilih objek **terdekat ke crosshair tengah**, init **NanoTrack** tracker dengannya.
2. **YOLO11 full-frame dimatikan** — hanya tracker yang jalan (jauh lebih ringan & stabil, tak kedap-kedip karena confidence YOLO).
3. Tiap frame: `tracker.update()` (KPU) → posisi target dihaluskan **filter alpha-beta** (prediksi kecepatan, `vx/vy`).
4. **Kalau target hilang** → **COAST** (pakai prediksi) lalu **auto re-lock**: jalankan YOLO sesaat, cari objek **sekelas** dalam radius prediksi (`RELOCK_RADIUS`), kunci ulang otomatis.
5. **Tidak otomatis balik ke YOLO** walau target hilang — tetap di mode LOCK **sampai operator mematikan tombol** (sesuai desain). Begitu `LOCK 0` diterima → `track.active=False` → balik ke SCAN.

**HUD taktis** (`draw_tactical`): corner-bracket reticle, garis crosshair→target, panah arah gerak (lead), bar confidence, dan warna status:
| Status | Warna | Arti |
|---|---|---|
| `LOCKED` | 🔴 merah | Target terkunci & terlihat |
| `RE-LOCK` | 🟡 kuning | Baru saja dikunci ulang otomatis |
| `SEARCH` | 🟠 oranye | Target hilang, sedang COAST + cari ulang |

### NanoTrack itu apa? (`nanotrack_kpu.py`)
Tracker Siamese single-object, **3 kmodel semua di KPU**:
| kmodel | Input | Output | Kapan |
|---|---|---|---|
| `cropped_test127` | template 127×127 | fitur [1,48,8,8] | sekali saat LOCK |
| `nanotrack_backbone_sim` | search 255×255 | fitur [1,48,16,16] | tiap frame |
| `nanotracker_head_calib_k230` | 2 fitur di atas | score + box (16×16 grid) | tiap frame |

Hasil di-*decode* (softmax, anchor, penalty, hanning window, argmax) port dari referensi C++ Canaan. **CPU hanya jadi "lem"** (crop/resize ringan + decode numpy ~2 ms); inti berat ada di KPU. Total ~7 ms/frame (~140 fps headroom).

---

## 6. Alur Perintah LOCK (GCS → Rover)

LOCK bisa dipicu **dua cara** (OR), keduanya dikirim ke rover lewat **uplink RF**:
1. **Tombol GCS** — `btn_lock` di `debug.py` (checkable: `[O] LOCK TARGET` ↔ `[#] TARGET LOCKED`).
2. **Switch SW5** di ESP32 controller (`SpacebotData.sw5`).

```
debug.py: send_lock_state()  (tiap 200 ms, berbasis STATE bukan edge → tahan paket hilang)
   active = btn_lock.isChecked()  OR  sw5
   kirim "LOCK 1" / "LOCK 0"  →  UDP 127.0.0.1:5601
        └─ rx_video.sh: wfb_tx -p1 -u5601  →  ►RF◄
   ──────────────────────────────────────────────
   tx_video.sh (RPi): wfb_rx -p1 -u5601 -c 192.168.88.1  →  teruskan UDP ke K230:5601
        └─ spacebot_ai.py: control_thread (UDP 5601)  →  lock_ctl["requested"] = True/False
```
Dikirim **berulang sebagai STATE** (bukan sekali/edge) supaya tahan terhadap paket yang hilang di link RF.

---

## 7. Referensi Port & Channel

| Apa | Protokol/Port | Sisi | Catatan |
|---|---|---|---|
| Video H264 K230→RPi | TCP **8554** | K230 listen | `tcp_nodelay=1` |
| Video RPi→GCS | UDP **5600** (mpegts) | via WFB **-p0** | hanya **1 proses** boleh bind 5600 |
| Perintah LOCK | UDP **5601** | via WFB **-p1** | "LOCK 1"/"LOCK 0" state |
| Telemetry AI (JSON) | TCP **5555** | K230 | **belum** dipakai debug.py |
| Telemetry+kontrol robot | USB serial **115200** | ESP32↔GCS | protokol `$DATA` 28 field |
| RF channel | **7** | WFB-ng | tx & rx **wajib simetris** |
| Akses K230 | SSH `root@192.168.88.1` | via RPi USB RNDIS | |

---

## 8. Struktur `debug.py` (GCS, single-file PyQt5)

Semua fitur di-*gate* oleh flag `GUI_AVAILABLE` / `OPENCV_AVAILABLE` / `MPV_AVAILABLE` agar file tetap bisa di-import walau ada dependensi yang hilang.

| Kelas | Fungsi |
|---|---|
| `SpacebotData` | Parser telemetry 28-field |
| `SerialHandler` / `SerialSignals` | Thread serial (baca ESP32) |
| `VideoStreamingWidget` | Video via **libmpv embed** + OSD ASS + **double-click fullscreen** |
| `OfflineMapWidget` | Peta OSM offline dari `map_tiles/`, trail + marker home |
| `JoystickWidget` / `AttitudeWidget` / `CompassWidget` / `BigValueWidget` | HUD |
| `AlarmSystem` | Bunyi alarm (aplay di Linux / winsound di Windows) |
| `SpacebotGCS(QMainWindow)` | Jendela utama; `update_timer` (50 ms) → `update_display()` |

`gcs_config.json` menyimpan ambang alarm & trim/EPA servo per channel (`SERVO_NUM_CH = 9`).

**Fitur fullscreen video:** double-click pada video → fullscreen 1 monitor; double-click / `Esc` → kembali ke panel. Karena mpv memiliki window native, double-click ditangkap binding mpv (`MBTN_LEFT_DBL`) → sinyal Qt. Saat toggle, mpv **dibongkar-pasang ulang** (reparent X11 mengubah winId) → ada *blip* singkat tapi tak freeze/black.

---

## 9. Cara Menjalankan & Deploy

### GCS (dua terminal, venv aktif)
```bash
cd ~/esp32_alarm && source venv/bin/activate
./rx_video.sh                 # terminal 1: wfb_rx -> UDP 5600  +  wfb_tx -p1 (uplink lock)
python3 debug.py              # terminal 2: GUI; login 1234, lalu klik START STREAM
```

### Rover
```bash
# K230 (via SSH root@192.168.88.1): salin spacebot_ai.py + nanotrack_kpu.py ke /root/, restart program AI
# RPi 4:
./tx_video.sh
```

### Cek sintaks setelah edit (tak ada build system/test)
```bash
python -m py_compile debug.py
python3 test_mpv.py                         # uji embedding mpv saja (tanpa stream)
python3 test_mpv.py udp://127.0.0.1:5600    # uji mpv dengan stream live
```

> `LOCKED_FINAL_20260609/` = snapshot **golden** (versi terakhir yang terbukti jalan).
> File `*.bak` = backup titik-waktu.

---

## 10. Jebakan Penting (pernah bikin gagal berulang)

Detail lengkap di `CLAUDE.md` & `SPACEBOT_WFBng_Session_Summary.md`. Ringkasnya:

- **Wajib jalan di venv** GCS (python-mpv ada di sana). Di luar venv, video diam-diam tak muncul.
- **libmpv butuh `LC_NUMERIC="C"`** atau segfault. `QApplication` mereset locale → `setlocale` dipanggil ulang di `main()` dan tepat sebelum `mpv.MPV(...)`. **Jangan dihapus.**
- **mpv dibuat lazy** (di `start_stream`, bukan `__init__`) — sebelum window realized → crash.
- Paket distro = **`libmpv1`** (Ubuntu 22.04 / Mint 21), bukan `libmpv2`.
- **Bug PPS encoder K230** permanen (`non-existing PPS 1`). Decode harus toleran: `hwdec=no` + `demuxer-lavf-o=fflags=+nobuffer+discardcorrupt`.
- **wfb_tx & wfb_rx wajib simetris** (sama `-p`, `-u`, key, FEC) — beda sedikit → RX tak terima apa-apa.
- **Hanya 1 proses** boleh bind UDP:5600 (`rx_video.sh` = wfb_rx saja; display milik `debug.py`). Jangan jalankan `ffplay` juga.

---

## 11. Peta File

```
debug.py                     GCS — aplikasi PyQt5 (video, peta, HUD, telemetry, tombol LOCK)
rx_video.sh                  GCS — wfb_rx video + wfb_tx uplink lock
spacebot_ai.py               K230 — YOLO11n + NanoTrack + encode + stream + control
nanotrack_kpu.py             K230 — tracker NanoTrack full-KPU
nano_profile.py              K230 — alat profiling waktu tiap tahap tracker
tx_video.sh                  RPi 4 — relay TCP→WFB video + relay uplink lock ke K230
test_mpv.py                  GCS — uji isolasi embedding libmpv
gcs_config.json              GCS — simpan ambang alarm & trim/EPA servo
*.kmodel                     model KPU (yolo11n + 3 NanoTrack)
CLAUDE.md                    panduan untuk tooling/AI
SPACEBOT_WFBng_Session_Summary.md   referensi kanonik hardware/WFB/bug/bitrate
DOKUMENTASI_SISTEM.md        (file ini) penjelasan sistem lengkap
LOCKED_FINAL_20260609/       snapshot golden (versi terbukti jalan)
*.bak                        backup titik-waktu
```

---

*Dokumen ini menjelaskan arsitektur & alur. Untuk angka hasil uji, wiring pin, dan setup key WFB, lihat `SPACEBOT_WFBng_Session_Summary.md`.*
