# 🛰️ SPACEBOT — Panduan Visual

Penjelasan sistem SPACEBOT dalam **diagram**. Semua diagram di bawah otomatis
ter-render saat dibuka di **GitHub** (Mermaid). Untuk penjelasan teks lengkap, lihat
[`DOKUMENTASI_SISTEM.md`](DOKUMENTASI_SISTEM.md).

---

## 1. 🗺️ Peta Besar — Tiga Mesin

Rover (di udara) ⟷ GCS (di darat). Video turun lewat RF, perintah LOCK naik lewat RF,
kontrol & telemetry robot lewat kabel serial.

```mermaid
graph LR
    subgraph ROVER["🚀 ROVER (udara)"]
        direction TB
        K230["<b>K230</b> — AI SoC<br/>spacebot_ai.py<br/>YOLO11n + NanoTrack<br/>+ encode H264"]
        RPI["<b>RPi 4</b> — Air Unit<br/>tx_video.sh<br/>ffmpeg copy + WFB"]
        K230 -->|"TCP 8554 H264<br/>(USB RNDIS)"| RPI
    end

    subgraph GCS["🖥️ GCS (darat)"]
        direction TB
        RX["rx_video.sh<br/>wfb_rx + wfb_tx"]
        APP["<b>debug.py</b><br/>PyQt5 GUI + libmpv"]
        ESP["🎮 ESP32 Controller<br/>stik · switch · telemetry"]
        RX -->|"UDP 5600"| APP
        ESP -->|"USB serial 115200"| APP
    end

    RPI ==>|"📡 RF ch.7 · -p0 VIDEO"| RX
    APP -.->|"perintah LOCK"| RX
    RX ==>|"📡 RF ch.7 · -p1 LOCK"| RPI

    classDef rover fill:#15233f,stroke:#4a90d9,color:#fff
    classDef gcs fill:#1f3b2a,stroke:#52c77a,color:#fff
    class K230,RPI rover
    class RX,APP,ESP gcs
```

---

## 2. 🎥 Pipeline Video (per langkah)

Kotak deteksi & HUD **digambar di K230**, lalu ikut ter-encode. RPi cuma menyalin
(tanpa encode ulang) → tambahan latency nyaris nol.

```mermaid
flowchart LR
    CAM["📷 Kamera<br/>640×480"] --> AI["K230: gambar kotak<br/>YOLO/NanoTrack + HUD"]
    AI --> ENC["ffmpeg h264_v4l2m2m<br/>300k CBR · -g10 · -bf0"]
    ENC -->|"TCP 8554"| COPY["RPi: ffmpeg -c:v copy<br/>→ MPEG-TS"]
    COPY -->|"UDP 5600"| WTX["wfb_tx -p0"]
    WTX ==>|"📡 RF ch.7"| WRX["wfb_rx -p0"]
    WRX -->|"UDP 127.0.0.1:5600"| MPV["debug.py: libmpv<br/>GPU + vsync"]
    MPV --> SCR["🖥️ Layar operator"]

    classDef k230 fill:#15233f,stroke:#4a90d9,color:#fff
    classDef rpi fill:#3f2f15,stroke:#d9a04a,color:#fff
    classDef gcs fill:#1f3b2a,stroke:#52c77a,color:#fff
    class CAM,AI,ENC k230
    class COPY,WTX rpi
    class WRX,MPV,SCR gcs
```

> 💡 **Kenapa mulus?** libmpv pakai GPU + vsync (semulus ffplay). **Kenapa low-latency?**
> `nobuffer`, `low_delay`, `tcp_nodelay`, `flush_packets`, GOP pendek, dan tak ada re-encode di RPi.

---

## 3. 🔀 Dua Jalur Data yang Terpisah Total

```mermaid
flowchart TB
    subgraph A["Jalur A — Kontrol & Telemetry Robot"]
        direction LR
        E["🎮 ESP32"] <-->|"USB serial 115200<br/>$DATA · 28 field"| D1["debug.py"]
    end
    subgraph B["Jalur B — Video (RF)"]
        direction LR
        KK["📷 K230"] -->|"WFB-ng RF"| D2["debug.py · libmpv"]
    end
    A ~~~ B

    classDef plane fill:#2a2333,stroke:#a06bd9,color:#fff
    class E,D1,KK,D2 plane
```

Keduanya **tidak dikorelasikan di software** — datang lewat jalan berbeda.
Telemetry AI K230 (JSON di TCP 5555) **belum** dipakai (rencana masa depan).

---

## 4. 🎯 Alur Perintah LOCK (GCS → Rover)

Tombol GCS **atau** switch SW5 → dikirim **berulang sebagai STATE** tiap 200 ms
(tahan paket hilang di RF).

```mermaid
sequenceDiagram
    autonumber
    actor U as 👤 Operator
    participant G as debug.py GCS
    participant TX as wfb_tx -p1 GCS
    participant RX as wfb_rx -p1 RPi
    participant K as spacebot_ai.py K230

    U->>G: Klik tombol LOCK / SW5 ON
    loop tiap 200 ms (berbasis STATE)
        G->>TX: UDP "LOCK 1" → :5601
        TX-->>RX: 📡 RF ch.7 (-p1)
        RX->>K: teruskan UDP → 192.168.88.1:5601
        K->>K: lock_ctl.requested = True
    end
    Note over K: Frame berikut → masuk mode TRACK
    U->>G: Matikan tombol / SW5 OFF
    G->>K: "LOCK 0" → kembali ke SCAN
```

---

## 5. 🧠 Mode Otak Rover: SCAN ↔ LOCK

Saat terkunci, **YOLO11 full-frame dimatikan** (hemat & stabil). Tidak otomatis
balik ke YOLO walau target hilang — **tetap LOCK sampai tombol dimatikan**.

```mermaid
stateDiagram-v2
    [*] --> SCAN
    SCAN: 🟢 SCAN — YOLO11n deteksi semua objek
    LOCKED: 🔴 LOCKED — NanoTrack, target terlihat
    SEARCH: 🟠 SEARCH — target hilang, COAST (prediksi)
    RELOCK: 🟡 RE-LOCK — dikunci ulang otomatis

    SCAN --> LOCKED: LOCK 1 + ada deteksi (terdekat crosshair)
    LOCKED --> SEARCH: target hilang
    SEARCH --> RELOCK: ketemu objek SEKELAS dalam radius
    RELOCK --> LOCKED: lanjut tracking
    SEARCH --> LOCKED: tracker dapat lagi
    LOCKED --> SCAN: LOCK 0 (tombol OFF)
    SEARCH --> SCAN: LOCK 0 (tombol OFF)
```

---

## 6. ⚙️ Loop Utama K230 (tiap frame)

```mermaid
flowchart TD
    A["📷 Frame BARU dari kamera"] --> B{"LOCK aktif?"}

    B -->|"Tidak"| C["SCAN: YOLO11n @224 di KPU"]
    C --> D["Gambar kotak hijau semua objek"]
    D --> E{"LOCK diminta<br/>& ada deteksi?"}
    E -->|"Ya"| F["Init NanoTrack ke objek<br/>terdekat crosshair → LOCKED"]
    E -->|"Tidak"| Z

    B -->|"Ya"| G["TRACK: NanoTrack.update() di KPU"]
    G --> H{"Target terlihat?"}
    H -->|"Ya"| I["Alpha-beta haluskan posisi<br/>🔴 LOCKED"]
    H -->|"Tidak"| J["COAST pakai prediksi<br/>🟠 SEARCH"]
    J --> K{"miss ≥ 4 &<br/>tiap 3 frame?"}
    K -->|"Ya"| L["Auto re-lock: YOLO cari sekelas<br/>dalam radius → 🟡 RE-LOCK"]
    K -->|"Tidak"| Z

    F --> Z["🎬 Encode H264 → stream ke GCS"]
    I --> Z
    L --> Z

    classDef scan fill:#1f3b2a,stroke:#52c77a,color:#fff
    classDef track fill:#3f1f1f,stroke:#d95252,color:#fff
    class C,D,E,F scan
    class G,H,I,J,K,L track
```

---

## 7. 🔬 Cara Kerja NanoTrack (full-KPU)

Tracker Siamese: bandingkan **template** (foto target saat dikunci) dengan area
**pencarian** tiap frame. **3 kmodel semua di KPU** — CPU hanya jadi "lem" ringan.

```mermaid
flowchart LR
    subgraph INIT["① Saat LOCK (sekali)"]
        A["Crop template<br/>127×127"] --> B["KPU: cropped_test127<br/>→ fitur [1,48,8,8]"]
    end
    subgraph UPD["② Tiap frame"]
        C["Crop search 255×255<br/>di sekitar prediksi"] --> D["KPU: backbone_sim<br/>→ fitur [1,48,16,16]"]
        B --> H
        D --> H["KPU: head<br/>→ score + box (grid 16×16)"]
        H --> DEC["Decode: softmax · anchor ·<br/>penalty · hanning · argmax"]
        DEC --> OUT["📦 Box target + skor"]
    end

    classDef kpu fill:#15233f,stroke:#4a90d9,color:#fff
    class A,B,C,D,H kpu
```

> ⏱️ **~7 ms/frame** (sekitar 140 fps headroom). KPU ≈ inti berat; CPU cuma crop/resize
> ringan + decode numpy. (Sub-window dioptimasi: 32 ms → 1.4 ms.)

---

## 8. 🎨 Legenda HUD Taktis (warna status)

Digambar di K230 (`draw_tactical`) dan menyatu di video:

| Warna | Status | Arti |
|:---:|---|---|
| 🔴 Merah | `LOCKED` | Target terkunci & terlihat |
| 🟡 Kuning | `RE-LOCK` | Baru dikunci ulang otomatis |
| 🟠 Oranye | `SEARCH` | Target hilang, sedang COAST + cari ulang |
| 🟢 Hijau | (SCAN) | Kotak deteksi YOLO biasa + panah arah gerak (lead) |

Elemen: corner-bracket reticle · center reticle · garis crosshair→target ·
panah lead (arah gerak) · bar confidence · teks `status conf dx dy`.

---

## 9. 🧩 Struktur `debug.py` (GCS)

```mermaid
graph TD
    M["<b>SpacebotGCS</b> (QMainWindow)<br/>update_timer 50 ms → update_display()"]
    M --> V["VideoStreamingWidget<br/>libmpv embed · OSD ASS · fullscreen"]
    M --> S["SerialHandler / SerialSignals<br/>SpacebotData.parse()"]
    M --> MAP["OfflineMapWidget<br/>tile OSM · trail · home"]
    M --> HUD["HUD widgets<br/>Joystick · Attitude · Compass · BigValue"]
    M --> AL["AlarmSystem<br/>aplay / winsound"]
    V -->|"udp://127.0.0.1:5600"| NET1["📡 video WFB"]
    S -->|"USB 115200"| NET2["🎮 ESP32"]

    classDef main fill:#2a2333,stroke:#a06bd9,color:#fff
    class M main
```

---

## 10. 🔌 Port & Channel (sekilas)

```mermaid
graph LR
    K230 -->|"TCP 8554"| RPI
    RPI -->|"UDP 5600 · WFB -p0"| GCS
    GCS -->|"UDP 5601 · WFB -p1"| K230
    ESP -->|"USB serial 115200"| GCS
    K230 -.->|"TCP 5555 JSON (belum dipakai)"| GCS

    classDef n fill:#1e1e28,stroke:#888,color:#ddd
    class K230,RPI,GCS,ESP n
```

| Apa | Port | Channel | Catatan |
|---|---|---|---|
| Video H264 K230→RPi | TCP 8554 | — | `tcp_nodelay=1` |
| Video RPi→GCS | UDP 5600 | WFB -p0 | hanya 1 proses bind 5600 |
| Perintah LOCK | UDP 5601 | WFB -p1 | "LOCK 1"/"LOCK 0" |
| Telemetry AI JSON | TCP 5555 | — | belum dipakai |
| Telemetry+kontrol robot | serial 115200 | — | `$DATA` 28 field |
| RF | — | **7** | tx & rx **wajib simetris** |

---

*Diagram = Mermaid (render otomatis di GitHub). Penjelasan teks: [`DOKUMENTASI_SISTEM.md`](DOKUMENTASI_SISTEM.md) · Hardware/WFB: [`SPACEBOT_WFBng_Session_Summary.md`](SPACEBOT_WFBng_Session_Summary.md).*
