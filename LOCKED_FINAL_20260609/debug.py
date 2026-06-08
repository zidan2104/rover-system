#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SPACEBOT GROUND CONTROL STATION v5.3 - REAL GPS & COMPASS

DATA FORMAT dari Controller v6.1:
$DATA|SWSTR|J1X|J1Y|J1B|J2X|J2Y|J2B|NRF_TX|NRF_OK|NRF_CONN|NRF_Q|NRF_RSSI|LORA|R|P|Y|V|I|W|PRES|ALT|TEMP|LAT|LON|SPD|SAT|HDG
  0     1    2   3   4   5   6   7   8    9      10      11   12       13  14 15 16 17 18 19  20  21   22  23  24  25  26  27

SWSTR    = 7 digit: SW1 SW2 SW3 SW4 SW5 BTN_MERAH BTN_HIJAU
NRF_CONN = 1 jika ACK diterima (robot terhubung), 0 jika tidak
NRF_Q    = kualitas sinyal NRF 0-100%
NRF_RSSI = estimasi RSSI dBm (-110 s/d -50)

AUTHOR: Zidan - SPACEBOT Project
"""

import sys
import serial
import serial.tools.list_ports
from datetime import datetime
from collections import deque
import threading
import time
import math
import os
import json
import socket

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gcs_config.json")

_CFG_DEFAULTS = {
    "temp_warn":  60.0,
    "temp_crit":  80.0,
    "volt_min":   9.9,
    "volt_max":   12.6,
    "volt_warn":  20,
    "volt_crit":  5,
    "pres_min":   0.0,
    "pres_max":   0.0,
    "pwr_warn":   0.0,
    "pwr_crit":   0.0,
}

SERVO_NUM_CH = 9  # channel PCA9685 yang bisa dikonfigurasi (0-8)
_SERVO_CH_DEFAULT = {"trim": 0, "epaL": 100, "epaR": 100}

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    print("[WARNING] OpenCV not found")
    OPENCV_AVAILABLE = False

# libmpv untuk video playback GPU-accelerated (smooth seperti ffplay)
# Install: sudo apt install libmpv2  (atau libmpv1)  +  pip install python-mpv
try:
    import mpv
    MPV_AVAILABLE = True
except Exception as _mpv_err:
    print("[WARNING] python-mpv / libmpv not available:", _mpv_err)
    MPV_AVAILABLE = False

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QGridLayout, QLabel, QFrame,
                                 QComboBox, QPushButton, QGroupBox, QProgressBar,
                                 QSplitter, QTextEdit, QCheckBox, QSpacerItem,
                                 QSizePolicy, QDialog, QLineEdit)

    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QRect, QThread, QPoint
    from PyQt5.QtGui import (QPainter, QColor, QPen, QFont, QBrush, QPolygon,
                             QPainterPath, QRadialGradient, QLinearGradient, QImage, QPixmap)
    GUI_AVAILABLE = True
except ImportError:
    print("[WARNING] PyQt5 not available")
    GUI_AVAILABLE = False


class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SYSTEM ACCESS")
        self.setFixedSize(400, 220)
        self.setWindowFlags(Qt.FramelessWindowHint)
        
        self.setStyleSheet("""
            QDialog { background-color: #0a0a0a; border: 2px solid #00ccff; border-radius: 5px; }
            QLabel { color: #00ccff; font-family: 'Consolas'; font-weight: bold; font-size: 14px; }
            QLineEdit { 
                background-color: #111; color: #00ccff; border: 1px solid #004466; 
                padding: 8px; font-family: 'Consolas'; font-size: 10px; border-radius: 3px;
            }
            QPushButton { 
                background-color: #1a1a1a; color: #00ccff; border: 1px solid #00ccff; 
                padding: 8px; font-family: 'Consolas'; font-weight: bold; border-radius: 3px;
            }
            QPushButton:hover { background-color: #002244; }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(40, 40, 40, 40)

        lbl_title = QLabel("SPACEBOT SECURE ACCESS")
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet("font-size: 18px; text-decoration: underline;")
        layout.addWidget(lbl_title)

        self.input_pass = QLineEdit()
        self.input_pass.setPlaceholderText("ENTER PASSCODE")
        self.input_pass.setEchoMode(QLineEdit.Password)
        self.input_pass.setAlignment(Qt.AlignCenter)
        self.input_pass.returnPressed.connect(self.check_password)
        layout.addWidget(self.input_pass)

        btn_layout = QHBoxLayout()
        btn_login = QPushButton("LOGIN")
        btn_login.clicked.connect(self.check_password)
        
        btn_exit = QPushButton("ABORT")
        btn_exit.clicked.connect(self.reject)
        btn_exit.setStyleSheet("color: red; border-color: red;")
        
        btn_layout.addWidget(btn_login)
        btn_layout.addWidget(btn_exit)
        layout.addLayout(btn_layout)

    def check_password(self):
        if self.input_pass.text() == "1234":
            self.accept()
        else:
            self.input_pass.clear()
            self.input_pass.setPlaceholderText("ACCESS DENIED!")


class SpacebotData:
    def __init__(self):
        self.reset()
        
    def reset(self):
        self.sw1 = False
        self.sw2 = False
        self.sw3 = False
        self.sw4 = False
        self.sw5 = False
        self.btn_merah = False
        self.btn_hijau = False
        
        self.j1x = 2048
        self.j1y = 2048
        self.j1btn = False
        self.j2x = 2048
        self.j2y = 2048
        self.j2btn = False
        
        self.nrf_tx = 0
        self.nrf_ok = 0
        self.nrf_rate = 0
        self.nrf_conn = False
        self.nrf_quality = 0    # 0-100%
        self.nrf_rssi = -110    # estimasi dBm

        self.lora_ok = False
        self.lora_last_update = 0
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.voltage = 0.0
        self.current = 0.0
        self.power = 0.0
        
        self.pressure = 0.0
        self.altitude = 0.0
        self.temperature = 0.0
        
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_speed = 0.0
        self.gps_satellites = 0
        self.gps_valid = False
        
        self.compass_heading = 0.0
        self.compass_valid = False
        self.has_heading_data = False

        self.last_update = 0
        self.packet_count = 0
        self.raw_line = ""
        
    def parse(self, line):
        try:
            self.raw_line = line
            
            if not line.startswith('$DATA|'):
                return False
                
            parts = line.strip().split('|')
            
            # Format v6.1 PASTI memiliki 28 elemen (index 0 - 27). 
            # Jika kurang, berarti paket korup/terpotong, buang saja.
            if len(parts) != 28:
                return False

            sw_str = parts[1]
            if len(sw_str) >= 5:
                self.sw1 = sw_str[0] == '1'
                self.sw2 = sw_str[1] == '1'
                self.sw3 = sw_str[2] == '1'
                self.sw4 = sw_str[3] == '1'
                self.sw5 = sw_str[4] == '1'
            if len(sw_str) >= 7:
                self.btn_merah = sw_str[5] == '1'
                self.btn_hijau = sw_str[6] == '1'

            self.j1x = int(parts[2])
            self.j1y = int(parts[3])
            self.j1btn = parts[4] == '1'
            self.j2x = int(parts[5])
            self.j2y = int(parts[6])
            self.j2btn = parts[7] == '1'

            self.nrf_tx = int(parts[8])
            self.nrf_ok = int(parts[9])
            self.nrf_rate = (self.nrf_ok * 100 // self.nrf_tx) if self.nrf_tx > 0 else 0
            self.nrf_conn = parts[10] == '1'

            # --- PARSING DATA SENSOR (Tanpa offset 'o') ---
            self.nrf_quality = int(float(parts[11]))
            self.nrf_rssi    = int(float(parts[12]))
            
            self.lora_ok = parts[13] == '1'
            if self.lora_ok:
                self.lora_last_update = time.time()

            self.roll  = float(parts[14])
            self.pitch = float(parts[15])
            self.yaw   = float(parts[16])

            self.voltage = float(parts[17])
            self.current = float(parts[18])
            self.power   = float(parts[19])

            self.pressure    = float(parts[20])
            self.altitude    = float(parts[21])
            self.temperature = float(parts[22])

            self.gps_lat        = float(parts[23])
            self.gps_lon        = float(parts[24])
            self.gps_speed      = float(parts[25])
            self.gps_satellites = int(parts[26])

            self.gps_valid = (
                self.gps_lat != 0.0 and
                self.gps_lon != 0.0 and
                self.gps_satellites > 0
            )

            hdg = float(parts[27])
            if hdg < 0:
                self.compass_valid    = False
                self.compass_heading  = 0.0
                self.has_heading_data = False
            elif hdg > 360:
                self.compass_valid    = False
                self.compass_heading  = hdg - 500.0
                self.has_heading_data = True
            else:
                self.compass_valid    = True
                self.compass_heading  = hdg
                self.has_heading_data = True
            
            self.last_update = time.time()
            self.packet_count += 1
            return True
            
        except Exception:
            return False


class SerialSignals(QObject):
    data_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

class SerialHandler:
    def __init__(self):
        self.port = None
        self.serial = None
        self.running = False
        self.thread = None
        self.signals = SerialSignals() if GUI_AVAILABLE else None
        self.data = SpacebotData()
        self.raw_lines = deque(maxlen=100)
        
    @staticmethod
    def list_ports():
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description,
                'hwid': port.hwid
            })
        return ports
    
    def connect(self, port, baudrate=115200):
        try:
            # TIMEOUT DIUBAH MENJADI 0.5 AGAR DATA TIDAK TERPOTONG
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
            self.port = port
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            if self.signals:
                self.signals.connection_changed.emit(True)
            return True
        except Exception as e:
            if self.signals:
                self.signals.error_occurred.emit(str(e))
            return False
    
    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
        if self.signals:
            self.signals.connection_changed.emit(False)
    
    def _read_loop(self):
        while self.running and self.serial:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.raw_lines.append(line)
                        self.data.parse(line)
                        if self.signals:
                            self.signals.data_received.emit(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.signals:
                    self.signals.error_occurred.emit(str(e))
                break
                
    def send(self, text):
        """Kirim string ke serial (ditambah newline otomatis)."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write((text + '\n').encode('utf-8'))
                return True
            except Exception:
                pass
        return False

    @property
    def is_connected(self):
        return self.serial is not None and self.serial.is_open

class VideoStreamingWidget(QWidget):
    """Video FPV via libmpv (GPU decode/scale/present + vsync) — smooth seperti ffplay.
    OSD (FPS / LIVE / crosshair / status) digambar lewat OSD internal mpv (ASS),
    jadi selalu tampil di atas video tanpa masalah native-window overlay Qt."""

    STREAM_URL = "udp://127.0.0.1:5600?overrun_nonfatal=1&fifo_size=5000000"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setStyleSheet("background-color: #050505;")

        # Frame native tempat mpv merender
        self.video_frame = QWidget(self)
        self.video_frame.setStyleSheet("background-color: #050505;")
        self.video_frame.setAttribute(Qt.WA_DontCreateNativeAncestors)
        self.video_frame.setAttribute(Qt.WA_NativeWindow)

        # -- State publik (dipakai main window) --------------------------
        self.mpv = None
        self.is_streaming = False
        self.status_text = "NO SIGNAL"
        self.video_fps = 0
        self.is_recording = False
        self.record_filename = ""
        self.record_frame_count = 0
        # ----------------------------------------------------------------

        self._rec_start   = 0.0
        self._ever_live   = False
        self._stall_since = None

        # JANGAN buat mpv di sini — window belum tampil, winId belum siap → crash.
        # mpv dibuat lazy saat start_stream() (window sudah pasti realized).
        # Placeholder digambar oleh paintEvent (Qt) selama mpv masih None.

        # Poll status + refresh OSD (ringan, 4x/detik)
        self._poll = QTimer(self)
        self._poll.timeout.connect(self._poll_state)
        self._poll.start(250)

    # -- MPV lifecycle --------------------------------------------------
    def _ensure_mpv(self):
        if self.mpv is not None or not MPV_AVAILABLE:
            return
        try:
            # WAJIB: libmpv segfault kalau LC_NUMERIC bukan "C".
            # QApplication mereset locale ke locale sistem (mis. id_ID pakai koma
            # desimal), jadi harus di-set ulang ke "C" tepat sebelum init mpv.
            import locale
            locale.setlocale(locale.LC_NUMERIC, 'C')

            self.mpv = mpv.MPV(
                wid=str(int(self.video_frame.winId())),
                vo='gpu',
                hwdec='no',                 # software decode: tahan bug PPS K230
                profile='low-latency',      # preset latency rendah
                osc=False,                  # tanpa on-screen controller
                border=False,
                idle='yes',                 # tetap hidup tanpa file
                force_window='yes',         # tampilkan window (hitam) walau idle
                keep_open='no',
                cursor_autohide='no',
                input_default_bindings=False,
                input_vo_keyboard=False,
                terminal=False,
                # ── LOW LATENCY (realtime) ──────────────────────────────
                cache='no',                 # matikan cache mpv
                cache_secs=0,
                demuxer_readahead_secs=0,   # jangan baca-maju
                audio='no',                 # tak ada audio → tak nunggu sync
                framedrop='vo',             # buang frame telat, selalu terbaru
                video_latency_hacks='yes',  # trik latency rendah mpv
                # ────────────────────────────────────────────────────────
            )
            # Robust terhadap stream rusak + low latency
            self.mpv['demuxer-lavf-o'] = 'fflags=+nobuffer+discardcorrupt'
            self.mpv['untimed'] = True      # tampil frame ASAP (mirip setpts=0/sync ext)
            # Zoom isi panel: video 4:3 di panel lebih lebar → buang pillarbox
            # (hitam kiri-kanan). panscan 1.0 = penuh (crop atas-bawah), tanpa distorsi.
            self.mpv['panscan'] = 1.0
        except Exception as e:
            print("[VIDEO] MPV init failed:", e)
            self.mpv = None
            self.status_text = "MPV INIT FAIL"

    def start_stream(self):
        if self.is_streaming:
            return
        if not MPV_AVAILABLE:
            self.status_text = "INSTALL python-mpv"
            self.update()
            return
        self._ensure_mpv()
        if self.mpv is None:
            self.update()
            return
        self._ever_live   = False
        self._stall_since = None
        self.is_streaming = True
        self.status_text  = "Connecting..."
        try:
            self.mpv.play(self.STREAM_URL)
        except Exception as e:
            print("[VIDEO] play error:", e)
            self.status_text = "MPV ERROR"
            self.is_streaming = False
        self._update_osd()

    def stop_stream(self):
        if self.is_recording:
            self.stop_recording()
        self.is_streaming = False
        self.status_text  = "NO SIGNAL"
        self.video_fps    = 0
        self._ever_live   = False
        if self.mpv is not None:
            try:
                self.mpv.command('stop')
            except Exception:
                pass
        self._update_osd()

    # -- Recording (stream-record mpv: dump .ts tanpa re-encode) --------
    def start_recording(self):
        if self.mpv is None or self.is_recording or not self.is_streaming:
            return False
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.record_filename = "rec_%s.ts" % ts
        try:
            self.mpv['stream-record'] = self.record_filename
        except Exception as e:
            print("[VIDEO] record start error:", e)
            return False
        self.is_recording = True
        self._rec_start = time.time()
        self.record_frame_count = 0
        return True

    def stop_recording(self):
        if self.mpv is not None and self.is_recording:
            try:
                self.mpv['stream-record'] = ''
            except Exception:
                pass
        self.is_recording = False

    # -- Poll status dari mpv -------------------------------------------
    def _poll_state(self):
        if self.is_streaming and self.mpv is not None:
            # Liveness pakai core-idle (andal walau untimed=True).
            # core-idle=False artinya mpv sedang aktif decode/render.
            try:
                idle = bool(self.mpv.core_idle)
            except Exception:
                idle = True

            # Angka FPS: estimated-vf-fps kadang 0 saat untimed → fallback container-fps
            try:
                fps = self.mpv.estimated_vf_fps or 0
            except Exception:
                fps = 0
            if not fps:
                try:
                    fps = self.mpv.container_fps or 0
                except Exception:
                    fps = 0
            self.video_fps = int(round(fps))

            if not idle:
                self.status_text  = "Live"
                self._ever_live   = True
                self._stall_since = None
            else:
                if self._stall_since is None:
                    self._stall_since = time.time()
                stalled = time.time() - self._stall_since
                if not self._ever_live:
                    self.status_text = "Connecting..." if stalled < 10 else "No Signal"
                else:
                    # pertahankan "Live" saat stall singkat agar tidak berkedip
                    self.status_text = "Signal Lost" if stalled > 3 else "Live"

            if self.is_recording:
                self.record_frame_count = int(
                    (time.time() - self._rec_start) * max(self.video_fps, 25))

        if self.mpv is not None:
            self._update_osd()       # OSD via mpv
        else:
            self.update()            # repaint placeholder Qt

    # -- OSD via ASS overlay mpv ----------------------------------------
    def _build_ass(self):
        # Kanvas virtual 1280x720 (mpv auto-scale ke ukuran video)
        parts = []
        # Crosshair "+" cyan semi-transparan di tengah
        parts.append(
            r"{\an5\pos(640,360)\fs46\bord1\3c&H000000&\1c&HFFC800&\alpha&H70&}+")

        if self.is_streaming and self.status_text == "Live":
            # FPS hanya ditampilkan kalau angkanya valid (>0)
            if self.video_fps > 0:
                if self.video_fps >= 20:
                    col = "&H50FF50&"   # hijau
                elif self.video_fps >= 10:
                    col = "&H00C8FF&"   # amber
                else:
                    col = "&H5050FF&"   # merah
                parts.append(
                    r"{\an7\pos(18,12)\fs24\bord2\3c&H000000&\1c" + col + r"}"
                    + ("%d FPS" % self.video_fps))
            parts.append(
                "{\\an9\\pos(1262,12)\\fs24\\bord2\\3c&H000000&\\1c&H3C3CFF&}● LIVE")
        else:
            parts.append(
                r"{\an5\pos(640,300)\fs30\bord2\3c&H000000&\1c&HB0A080&}"
                + str(self.status_text))
        return "\n".join(parts)

    def _update_osd(self):
        if self.mpv is None:
            return
        try:
            self.mpv.command('osd-overlay', 0, 'ass-events',
                             self._build_ass(), 1280, 720)
        except Exception:
            pass

    # -- Geometry / cleanup ---------------------------------------------
    def resizeEvent(self, event):
        self.video_frame.setGeometry(0, 0, self.width(), self.height())
        super().resizeEvent(event)

    def paintEvent(self, event):
        # Hanya dipakai saat mpv tidak ada (native window mpv menutupi widget)
        if self.mpv is not None:
            return
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(5, 5, 5))
        p.setPen(QPen(QColor(0, 200, 255), 2))
        p.drawRect(0, 0, self.width() - 1, self.height() - 1)
        p.setPen(QColor(255, 120, 120))
        p.setFont(QFont("Consolas", 12, QFont.Bold))
        p.drawText(self.rect(), Qt.AlignCenter, self.status_text)
        p.end()

    def closeEvent(self, event):
        try:
            if self.mpv is not None:
                self.mpv.terminate()
                self.mpv = None
        except Exception:
            pass
        super().closeEvent(event)


class OfflineMapWidget(QWidget):
    position_clicked = pyqtSignal(float, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.zoom = 17
        self.min_zoom = 2
        self.max_zoom = 19
        
        # Default center: Operation Area
        self.center_lat = -6.399617
        self.center_lon = 106.91176
        
        self.drone_lat = self.center_lat
        self.drone_lon = self.center_lon
        self.drone_heading = 0.0
        self.drone_altitude = 0.0
        self.drone_speed = 0.0
        self.show_drone = True
        
        self.gps_valid = False
        self.gps_satellites = 0
        
        self.auto_center = True
        
        self.trail_points = []
        self.max_trail_points = 200
        self.show_trail = True
        
        self.home_lat = self.center_lat
        self.home_lon = self.center_lon
        self.home_set = False
        self.show_home = True
        
        self.offset_x = 0
        self.offset_y = 0
        
        self.dragging = False
        self.last_mouse_pos = None
        
        self.tile_size = 256
        self.tile_dir = "map_tiles"
        self.tile_cache = {}
        
        self.setMinimumSize(200, 200)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
    
    def lat_lon_to_tile(self, lat, lon, zoom):
        n = 2 ** zoom
        x = int((lon + 180.0) / 360.0 * n)
        lat_rad = math.radians(lat)
        y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return x, y
    
    def lat_lon_to_pixel(self, lat, lon):
        n = 2 ** self.zoom
        center_pixel_x = (self.center_lon + 180.0) / 360.0 * n * self.tile_size
        center_pixel_y = (1.0 - math.asinh(math.tan(math.radians(self.center_lat))) / math.pi) / 2.0 * n * self.tile_size
        
        target_pixel_x = (lon + 180.0) / 360.0 * n * self.tile_size
        lat_rad = math.radians(lat)
        target_pixel_y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n * self.tile_size
        
        rel_x = target_pixel_x - center_pixel_x
        rel_y = target_pixel_y - center_pixel_y
        
        widget_x = self.width() // 2 + rel_x + self.offset_x
        widget_y = self.height() // 2 + rel_y + self.offset_y
        
        return int(widget_x), int(widget_y)
    
    def pixel_to_lat_lon(self, px, py):
        n = 2 ** self.zoom
        center_pixel_x = (self.center_lon + 180.0) / 360.0 * n * self.tile_size
        center_pixel_y = (1.0 - math.asinh(math.tan(math.radians(self.center_lat))) / math.pi) / 2.0 * n * self.tile_size
        
        world_x = center_pixel_x + (px - self.width() // 2 - self.offset_x)
        world_y = center_pixel_y + (py - self.height() // 2 - self.offset_y)
        
        lon = world_x / (n * self.tile_size) * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * world_y / (n * self.tile_size))))
        lat = math.degrees(lat_rad)
        
        return lat, lon
    
    def get_tile_path(self, x, y, z):
        return os.path.join(self.tile_dir, str(z), str(x), str(y) + ".png")
    
    def load_tile(self, x, y, z):
        cache_key = (x, y, z)
        if cache_key in self.tile_cache:
            return self.tile_cache[cache_key]
        
        tile_path = self.get_tile_path(x, y, z)
        if os.path.exists(tile_path):
            pixmap = QPixmap(tile_path)
            if not pixmap.isNull():
                self.tile_cache[cache_key] = pixmap
                return pixmap
        return None
    
    def set_drone_position(self, lat, lon, heading=None, altitude=None, speed=None, 
                           gps_valid=False, satellites=0):
        if gps_valid and lat != 0.0 and lon != 0.0:
            self.drone_lat = lat
            self.drone_lon = lon
            self.gps_valid = True
            self.gps_satellites = satellites
            
            if not self.home_set:
                self.home_lat = lat
                self.home_lon = lon
                self.home_set = True
            
            if self.show_trail:
                if len(self.trail_points) == 0:
                    self.trail_points.append((lat, lon))
                else:
                    last_lat, last_lon = self.trail_points[-1]
                    dist = math.sqrt((lat - last_lat)**2 + (lon - last_lon)**2)
                    if dist > 0.00001:
                        self.trail_points.append((lat, lon))
                        if len(self.trail_points) > self.max_trail_points:
                            self.trail_points.pop(0)
            
            if self.auto_center:
                self.center_lat = lat
                self.center_lon = lon
                self.offset_x = 0
                self.offset_y = 0
        else:
            self.gps_valid = False
            self.gps_satellites = satellites
        
        if heading is not None:
            self.drone_heading = heading
            
        if altitude is not None:
            self.drone_altitude = altitude
            
        if speed is not None:
            self.drone_speed = speed
        
        self.update()
    
    def set_home_position(self, lat, lon):
        self.home_lat = lat
        self.home_lon = lon
        self.home_set = True
        self.update()
    
    def center_on_drone(self):
        if self.gps_valid:
            self.center_lat = self.drone_lat
            self.center_lon = self.drone_lon
        self.offset_x = 0
        self.offset_y = 0
        self.update()
    
    def center_on_home(self):
        if self.home_set:
            self.center_lat = self.home_lat
            self.center_lon = self.home_lon
            self.offset_x = 0
            self.offset_y = 0
            self.update()
    
    def center_on_position(self, lat, lon):
        self.center_lat = lat
        self.center_lon = lon
        self.offset_x = 0
        self.offset_y = 0
        self.update()
    
    def toggle_auto_center(self):
        self.auto_center = not self.auto_center
        if self.auto_center:
            self.center_on_drone()
        self.update()
    
    def zoom_in(self):
        if self.zoom < self.max_zoom:
            self.zoom += 1
            self.offset_x = 0
            self.offset_y = 0
            self.update()
    
    def zoom_out(self):
        if self.zoom > self.min_zoom:
            self.zoom -= 1
            self.offset_x = 0
            self.offset_y = 0
            self.update()
    
    def clear_trail(self):
        self.trail_points = []
        self.update()
    
    def wheelEvent(self, event):
        self.auto_center = False
        delta = event.angleDelta().y()
        if delta > 0:
            self.zoom_in()
        elif delta < 0:
            self.zoom_out()
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.auto_center = False
            self.dragging = True
            self.last_mouse_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self.setCursor(Qt.ArrowCursor)
    
    def mouseMoveEvent(self, event):
        if self.dragging and self.last_mouse_pos:
            delta = event.pos() - self.last_mouse_pos
            self.offset_x += delta.x()
            self.offset_y += delta.y()
            self.last_mouse_pos = event.pos()
            self.update()
    
    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            lat, lon = self.pixel_to_lat_lon(event.x(), event.y())
            self.center_on_position(lat, lon)
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Plus or event.key() == Qt.Key_Equal:
            self.zoom_in()
        elif event.key() == Qt.Key_Minus:
            self.zoom_out()
        elif event.key() == Qt.Key_C:
            self.center_on_drone()
        elif event.key() == Qt.Key_A:
            self.toggle_auto_center()
        elif event.key() == Qt.Key_H:
            self.center_on_home()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w = self.width()
        h = self.height()
        
        painter.fillRect(0, 0, w, h, QColor(20, 25, 30))
        
        tiles_drawn = self.draw_tiles(painter)
        if not tiles_drawn:
            self.draw_grid(painter)
        
        if self.show_trail and len(self.trail_points) > 1:
            self.draw_trail(painter)
        
        if self.show_home and self.home_set:
            self.draw_home_marker(painter)
        
        if self.show_drone:
            self.draw_drone_marker(painter)
        
        self.draw_overlay(painter)
        
        if self.gps_valid:
            border_color = QColor(0, 255, 100)
        else:
            border_color = QColor(255, 100, 100)
        painter.setPen(QPen(border_color, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(0, 0, w-1, h-1)
    
    def draw_tiles(self, painter):
        w = self.width()
        h = self.height()
        
        center_tile_x, center_tile_y = self.lat_lon_to_tile(self.center_lat, self.center_lon, self.zoom)
        tiles_x = (w // self.tile_size) + 3
        tiles_y = (h // self.tile_size) + 3
        
        n = 2 ** self.zoom
        center_pixel_x = (self.center_lon + 180.0) / 360.0 * n * self.tile_size
        center_pixel_y = (1.0 - math.asinh(math.tan(math.radians(self.center_lat))) / math.pi) / 2.0 * n * self.tile_size
        tile_offset_x = center_pixel_x % self.tile_size
        tile_offset_y = center_pixel_y % self.tile_size
        
        tiles_drawn = False
        
        for dx in range(-tiles_x // 2, tiles_x // 2 + 1):
            for dy in range(-tiles_y // 2, tiles_y // 2 + 1):
                tile_x = center_tile_x + dx
                tile_y = center_tile_y + dy
                
                max_tile = 2 ** self.zoom
                tile_x = tile_x % max_tile
                if tile_y < 0 or tile_y >= max_tile:
                    continue
                
                pixmap = self.load_tile(tile_x, tile_y, self.zoom)
                
                if pixmap and not pixmap.isNull():
                    px = w // 2 + dx * self.tile_size - tile_offset_x + self.offset_x
                    py = h // 2 + dy * self.tile_size - tile_offset_y + self.offset_y
                    painter.drawPixmap(int(px), int(py), pixmap)
                    tiles_drawn = True
        
        return tiles_drawn
    
    def draw_grid(self, painter):
        w = self.width()
        h = self.height()
        
        painter.setPen(QPen(QColor(0, 60, 100), 1))
        grid_pixels = 50
        for x in range(0, w + grid_pixels, grid_pixels):
            painter.drawLine(x, 0, x, h)
        for y in range(0, h + grid_pixels, grid_pixels):
            painter.drawLine(0, y, w, y)
        
        cx = w // 2 + self.offset_x
        cy = h // 2 + self.offset_y
        painter.setPen(QPen(QColor(0, 200, 255, 100), 1))
        painter.drawLine(cx - 20, cy, cx + 20, cy)
        painter.drawLine(cx, cy - 20, cx, cy + 20)
        
        meters_per_pixel = 156543.03 * math.cos(math.radians(self.center_lat)) / (2 ** self.zoom)
        grid_meters = meters_per_pixel * grid_pixels
        
        painter.setPen(QColor(100, 100, 120))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(10, h - 15, "[GRID] ~%.0fm/grid | Zoom:%d" % (grid_meters, self.zoom))
    
    def draw_trail(self, painter):
        if len(self.trail_points) < 2:
            return
            
        for i in range(len(self.trail_points) - 1):
            alpha = int(50 + (float(i) / len(self.trail_points)) * 200)
            painter.setPen(QPen(QColor(255, 200, 0, alpha), 2))
            
            lat1, lon1 = self.trail_points[i]
            lat2, lon2 = self.trail_points[i + 1]
            
            px1, py1 = self.lat_lon_to_pixel(lat1, lon1)
            px2, py2 = self.lat_lon_to_pixel(lat2, lon2)
            
            painter.drawLine(px1, py1, px2, py2)
    
    def draw_home_marker(self, painter):
        hx, hy = self.lat_lon_to_pixel(self.home_lat, self.home_lon)
        
        painter.setPen(QPen(QColor(0, 255, 0), 2))
        painter.setBrush(QBrush(QColor(0, 255, 0, 100)))
        
        house = QPolygon([
            QPoint(hx, hy - 12),
            QPoint(hx + 10, hy),
            QPoint(hx + 7, hy),
            QPoint(hx + 7, hy + 8),
            QPoint(hx - 7, hy + 8),
            QPoint(hx - 7, hy),
            QPoint(hx - 10, hy)
        ])
        painter.drawPolygon(house)
        
        painter.setPen(QColor(0, 255, 0))
        painter.setFont(QFont("Consolas", 8, QFont.Bold))
        painter.drawText(hx + 12, hy + 4, "HOME")
    
    def draw_drone_marker(self, painter):
        dx, dy = self.lat_lon_to_pixel(self.drone_lat, self.drone_lon)
        
        painter.save()
        painter.translate(dx, dy)
        painter.rotate(self.drone_heading)
        
        if self.gps_valid:
            drone_color = QColor(255, 200, 0)
            center_color = QColor(0, 255, 0)
        else:
            drone_color = QColor(255, 100, 100)
            center_color = QColor(255, 0, 0)
        
        painter.setPen(QPen(drone_color, 2))
        painter.setBrush(QBrush(drone_color.lighter(120)))
        
        drone_shape = QPolygon([
            QPoint(0, -18),
            QPoint(-12, 12),
            QPoint(0, 5),
            QPoint(12, 12)
        ])
        painter.drawPolygon(drone_shape)
        
        painter.setBrush(QBrush(center_color))
        painter.drawEllipse(-4, -4, 8, 8)
        
        painter.restore()
        
        painter.setPen(drone_color)
        painter.setFont(QFont("Consolas", 8))
        info_text = "HDG:%.0f SPD:%.1fkm/h" % (self.drone_heading, self.drone_speed)
        painter.drawText(dx + 18, dy - 5, info_text)
    
    def draw_overlay(self, painter):
        w = self.width()
        h = self.height()
        
        painter.fillRect(5, 5, 220, 75, QColor(0, 0, 0, 200))
        
        painter.setFont(QFont("Consolas", 9))
        
        if self.gps_valid:
            painter.setPen(QColor(0, 255, 100))
            gps_status = "GPS: VALID (%d sats)" % self.gps_satellites
        else:
            painter.setPen(QColor(255, 100, 100))
            gps_status = "GPS: NO FIX (%d sats)" % self.gps_satellites
        painter.drawText(10, 18, gps_status)
        
        painter.setPen(QColor(0, 200, 255))
        painter.drawText(10, 32, "POS: %.6f, %.6f" % (self.drone_lat, self.drone_lon))
        
        painter.drawText(10, 46, "HDG: %.1f | ALT: %.1fm" % (self.drone_heading, self.drone_altitude))
        
        if self.auto_center:
            ac_color = QColor(0, 255, 100)
            ac_text = "AUTO-CENTER: ON"
        else:
            ac_color = QColor(150, 150, 150)
            ac_text = "AUTO-CENTER: OFF"
        painter.setPen(ac_color)
        painter.drawText(10, 60, ac_text)
        
        painter.setPen(QColor(100, 100, 120))
        painter.drawText(10, 74, "[A]=Auto [C]=Center [H]=Home")
        
        btn_size = 28
        margin = 10
        
        painter.fillRect(w - btn_size - margin - 3, margin - 3, btn_size + 6, btn_size * 2 + 16, QColor(0, 0, 0, 180))
        
        painter.setPen(QPen(QColor(0, 200, 255), 2))
        painter.setBrush(QBrush(QColor(30, 40, 50)))
        painter.drawRect(w - btn_size - margin, margin, btn_size, btn_size)
        painter.setFont(QFont("Arial", 14, QFont.Bold))
        painter.drawText(w - btn_size - margin + 7, margin + 20, "+")
        
        painter.drawRect(w - btn_size - margin, margin + btn_size + 5, btn_size, btn_size)
        painter.drawText(w - btn_size - margin + 9, margin + btn_size + 25, "-")
        
        painter.setPen(QColor(0, 200, 255))
        painter.setFont(QFont("Consolas", 8))
        painter.drawText(w - btn_size - margin - 5, margin + btn_size * 2 + 20, "Z:%d" % self.zoom)


if GUI_AVAILABLE:
    
    class JoystickWidget(QWidget):
        def __init__(self, title="Joystick", parent=None):
            super().__init__(parent)
            self.title = title
            self.x_value = 2048
            self.y_value = 2048
            self.btn_pressed = False
            self.setMinimumSize(100, 100)
            
        def set_values(self, x, y, btn):
            self.x_value = x
            self.y_value = y
            self.btn_pressed = btn
            self.update()
            
        def paintEvent(self, event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            w = self.width()
            h = self.height()
            size = min(w, h) - 30
            cx = w // 2
            cy = h // 2 + 5
            radius = size // 2
            
            gradient = QRadialGradient(cx, cy, radius)
            gradient.setColorAt(0, QColor(50, 50, 60))
            gradient.setColorAt(1, QColor(20, 20, 30))
            painter.setPen(QPen(QColor(0, 200, 255, 100), 2))
            painter.setBrush(QBrush(gradient))
            painter.drawEllipse(cx - radius, cy - radius, size, size)
            
            painter.setPen(QPen(QColor(0, 200, 255, 50), 1))
            painter.drawLine(cx - radius, cy, cx + radius, cy)
            painter.drawLine(cx, cy - radius, cx, cy + radius)
            
            norm_v = (self.x_value - 2048) / 2048.0
            norm_h = (self.y_value - 2048) / 2048.0
            mag = (norm_h**2 + norm_v**2)**0.5
            if mag > 1.0:
                norm_h /= mag
                norm_v /= mag
            
            px = int(cx + norm_h * radius * 0.85)
            py = int(cy - norm_v * radius * 0.85)
            
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 0, 0, 100)))
            painter.drawEllipse(px - 11, py - 9, 24, 24)
            
            if self.btn_pressed:
                color = QColor(255, 80, 80)
            else:
                color = QColor(0, 200, 255)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawEllipse(px - 12, py - 12, 24, 24)
            
            painter.setPen(QColor(0, 200, 255))
            painter.setFont(QFont("Consolas", 10, QFont.Bold))
            painter.drawText(5, 15, self.title)
            
            painter.setFont(QFont("Consolas", 8))
            if self.x_value > 2200:
                up_down = "UP"
            elif self.x_value < 1800:
                up_down = "DN"
            else:
                up_down = "--"
            if self.y_value > 2200:
                left_right = "RT"
            elif self.y_value < 1800:
                left_right = "LT"
            else:
                left_right = "--"
            painter.drawText(5, h - 18, "X:%d Y:%d" % (self.x_value, self.y_value))
            btn_str = "BTN" if self.btn_pressed else ""
            painter.drawText(5, h - 5, "[%s %s] %s" % (up_down, left_right, btn_str))


    class AttitudeWidget(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.roll = 0.0
            self.pitch = 0.0
            self.setMinimumSize(100, 100)
            
        def set_attitude(self, roll, pitch):
            self.roll = max(-45, min(45, roll))
            self.pitch = max(-45, min(45, pitch))
            self.update()
            
        def paintEvent(self, event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            w = self.width()
            h = self.height()
            size = min(w, h) - 30
            cx = w // 2
            cy = h // 2
            radius = size // 2
            
            clip_path = QPainterPath()
            clip_path.addEllipse(cx - radius, cy - radius, size, size)
            painter.setClipPath(clip_path)
            painter.fillRect(0, 0, w, h, QColor(30, 30, 40))
            
            painter.save()
            painter.translate(cx, cy)
            painter.rotate(-self.roll)
            pitch_offset = int(-self.pitch * radius / 45)
            
            sky_gradient = QLinearGradient(0, -radius, 0, pitch_offset)
            sky_gradient.setColorAt(0, QColor(30, 80, 140))
            sky_gradient.setColorAt(1, QColor(70, 130, 180))
            painter.setBrush(QBrush(sky_gradient))
            painter.setPen(Qt.NoPen)
            painter.drawRect(int(-radius - 50), int(-radius - 50), int(size + 100), int(radius + pitch_offset + 50))
            
            ground_gradient = QLinearGradient(0, pitch_offset, 0, radius)
            ground_gradient.setColorAt(0, QColor(139, 90, 43))
            ground_gradient.setColorAt(1, QColor(80, 50, 25))
            painter.setBrush(QBrush(ground_gradient))
            painter.drawRect(int(-radius - 50), int(pitch_offset), int(size + 100), int(radius - pitch_offset + 50))
            
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawLine(int(-radius), int(pitch_offset), int(radius), int(pitch_offset))
            painter.restore()
            painter.setClipping(False)
            
            painter.setPen(QPen(QColor(0, 200, 255), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(cx - radius, cy - radius, size, size)
            
            painter.setPen(QPen(QColor(255, 200, 0), 3))
            painter.drawLine(cx - 35, cy, cx - 12, cy)
            painter.drawLine(cx + 12, cy, cx + 35, cy)
            painter.setBrush(QBrush(QColor(255, 200, 0)))
            painter.drawEllipse(cx - 4, cy - 4, 8, 8)
            
            painter.setPen(QColor(0, 200, 255))
            painter.setFont(QFont("Consolas", 9))
            painter.drawText(5, h - 5, "R:%+.1f P:%+.1f" % (self.roll, self.pitch))


    class CompassWidget(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.heading  = 0.0
            self.is_valid = False  # True=mag valid, False=IMU only atau no data
            self.has_heading = False  # True jika ada heading apapun (IMU atau mag)
            self.setMinimumSize(100, 100)

        def set_heading(self, heading, valid=True, has_heading=True):
            self.heading     = heading % 360
            self.is_valid    = valid
            self.has_heading = has_heading
            self.update()
            
        def paintEvent(self, event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            w = self.width()
            h = self.height()
            size = min(w, h) - 30
            cx = w // 2
            cy = h // 2
            radius = size // 2
            
            gradient = QRadialGradient(cx, cy, radius)
            gradient.setColorAt(0, QColor(40, 40, 50))
            gradient.setColorAt(1, QColor(20, 20, 30))
            
            if self.is_valid:
                border_color = QColor(0, 200, 255)
            else:
                border_color = QColor(255, 100, 100)
            painter.setPen(QPen(border_color, 2))
            painter.setBrush(QBrush(gradient))
            painter.drawEllipse(cx - radius, cy - radius, size, size)
            
            painter.save()
            painter.translate(cx, cy)
            painter.rotate(-self.heading)
            
            directions = [
                (0, 'N', QColor(255, 100, 100)),
                (90, 'E', QColor(200, 200, 200)),
                (180, 'S', QColor(200, 200, 200)),
                (270, 'W', QColor(200, 200, 200))
            ]
            for angle, label, color in directions:
                painter.save()
                painter.rotate(angle)
                painter.setPen(color)
                painter.setFont(QFont("Arial", 11, QFont.Bold))
                painter.drawText(-6, -radius + 18, label)
                painter.restore()
                
            for i in range(36):
                painter.save()
                painter.rotate(i * 10)
                if i % 9 == 0:
                    painter.setPen(QPen(QColor(0, 200, 255), 2))
                    painter.drawLine(0, -radius + 3, 0, -radius + 15)
                elif i % 3 == 0:
                    painter.setPen(QPen(QColor(150, 150, 180), 1))
                    painter.drawLine(0, -radius + 3, 0, -radius + 10)
                else:
                    painter.setPen(QPen(QColor(100, 100, 120), 1))
                    painter.drawLine(0, -radius + 3, 0, -radius + 7)
                painter.restore()
            painter.restore()
            
            painter.setPen(QPen(QColor(255, 200, 0), 2))
            painter.setBrush(QBrush(QColor(255, 200, 0)))
            triangle = QPolygon([
                QPoint(cx, cy - 18),
                QPoint(cx - 10, cy + 12),
                QPoint(cx, cy + 5),
                QPoint(cx + 10, cy + 12)
            ])
            painter.drawPolygon(triangle)
            
            painter.setPen(QColor(0, 200, 255))
            painter.setFont(QFont("Consolas", 9))
            if self.is_valid:
                status = "COMPASS"
            elif self.has_heading:
                status = "IMU HDG"
            else:
                status = "NO DATA"
            painter.drawText(5, 15, status)
            painter.drawText(5, h - 5, "HDG: %03.0f" % self.heading)


    class AlarmSystem:
        """Generates tones via aplay (no extra deps). Each alarm type has unique sound."""
        SAMPLE_RATE = 22050
        _muted = False
        _playing = False

        # --- continuous-loop state ---
        _loop_active = set()   # set of alarm_type currently looping
        _loop_thread = None

        # Priority: higher index = higher priority when multiple active
        _LOOP_PRIORITY = ['high_attitude', 'temp_crit', 'batt_crit', 'crit']

        # Human-readable label per alarm type (untuk ditampilkan di UI)
        _LABELS = {
            'high_attitude': 'ATTITUDE',
            'temp_crit':     'TEMP CRIT',
            'batt_crit':     'BATT CRIT',
            'crit':          'CRITICAL',
        }

        @classmethod
        def loop_start(cls, alarm_type):
            """Mulai memainkan alarm secara berulang tanpa jeda sampai loop_stop dipanggil."""
            cls._loop_active.add(alarm_type)
            if cls._loop_thread is None or not cls._loop_thread.is_alive():
                import threading as _t
                cls._loop_thread = _t.Thread(target=cls._loop_bg, daemon=True)
                cls._loop_thread.start()

        @classmethod
        def loop_stop(cls, alarm_type):
            """Hentikan looping untuk alarm_type tertentu."""
            cls._loop_active.discard(alarm_type)

        @classmethod
        def _loop_bg(cls):
            while cls._loop_active:
                if cls._muted:
                    time.sleep(0.05)
                    continue
                # Pilih alarm prioritas tertinggi yang sedang aktif
                alarm = None
                for a in reversed(cls._LOOP_PRIORITY):
                    if a in cls._loop_active:
                        alarm = a
                        break
                if alarm is None:
                    break
                cls._play_bg(alarm)

        @classmethod
        def set_mute(cls, muted):
            cls._muted = muted

        @staticmethod
        def _tone(freq, duration, vol=0.92, sr=22050):
            import struct, math
            n = int(sr * duration)
            buf = bytearray(n * 2)
            fade = min(int(sr * 0.01), n // 4)
            for i in range(n):
                env = min(1.0, i / max(fade, 1), (n - i) / max(fade, 1))
                s = int(32767 * vol * env * math.sin(2 * math.pi * freq * i / sr))
                struct.pack_into('<h', buf, i * 2, max(-32767, min(32767, s)))
            return bytes(buf)

        @staticmethod
        def _silence(duration, sr=22050):
            return bytes(int(sr * duration) * 2)

        @classmethod
        def _build(cls, alarm_type):
            t, s, sr = cls._tone, cls._silence, cls.SAMPLE_RATE
            if alarm_type == 'info':
                return t(880, 0.12, 0.92, sr)
            elif alarm_type == 'warn':
                return t(660, 0.14, 0.92, sr) + s(0.07, sr) + t(660, 0.14, 0.92, sr)
            elif alarm_type == 'crit':
                chunk = t(440, 0.09, 0.96, sr) + s(0.04, sr)
                return chunk * 5
            elif alarm_type == 'lora_lost':
                return b''.join(t(f, 0.11, 0.6, sr) for f in [800, 650, 500, 350])
            elif alarm_type == 'lora_ok':
                return b''.join(t(f, 0.09, 0.45, sr) for f in [440, 660])
            elif alarm_type == 'gps_lost':
                return t(520, 0.18, 0.95, sr) + s(0.09, sr) + t(520, 0.18, 0.95, sr)
            elif alarm_type == 'gps_ok':
                return b''.join(t(f, 0.08, 0.4, sr) for f in [440, 550, 660])
            elif alarm_type == 'batt_warn':
                return t(600, 0.14, 0.95, sr) + s(0.07, sr) + t(400, 0.22, 0.95, sr)
            elif alarm_type == 'batt_crit':
                chunk = t(480, 0.07, 0.98, sr) + s(0.035, sr)
                return chunk * 7
            elif alarm_type == 'temp_warn':
                return b''.join(t(f, 0.1, 0.5, sr) for f in [400, 520, 640])
            elif alarm_type == 'temp_crit':
                chunk = t(560, 0.08, 0.98, sr) + s(0.04, sr)
                return chunk * 5
            elif alarm_type == 'apogee':
                return b''.join(t(f, 0.11, 0.6, sr) for f in [440, 550, 660, 880])
            elif alarm_type == 'high_attitude':
                chunk = t(750, 0.1, 0.95, sr) + s(0.05, sr)
                return chunk * 3
            elif alarm_type == 'mission_start':
                return b''.join(t(f, 0.09, 0.5, sr) for f in [440, 550, 660, 770, 880])
            elif alarm_type == 'mission_stop':
                return b''.join(t(f, 0.09, 0.5, sr) for f in [880, 660, 440])
            return t(440, 0.15, 0.4, sr)

        @classmethod
        def play(cls, alarm_type):
            if cls._muted:
                return
            import threading
            threading.Thread(target=cls._play_bg, args=(alarm_type,), daemon=True).start()

        # Beep patterns per alarm type: list of (freq_hz, duration_ms)
        # freq=0 ? jeda (sleep)
        _BEEP_PATTERNS = {
            'info':          [(880, 120)],
            'warn':          [(660, 140), (0, 70), (660, 140)],
            'crit':          [(440, 90),  (0, 40)] * 5,
            'lora_lost':     [(800, 110), (650, 110), (500, 110), (350, 110)],
            'lora_ok':       [(440, 90),  (660, 90)],
            'gps_lost':      [(520, 180), (0, 90),  (520, 180)],
            'gps_ok':        [(440, 80),  (550, 80), (660, 80)],
            'batt_warn':     [(600, 140), (0, 70),  (400, 220)],
            'batt_crit':     [(480, 70),  (0, 35)]  * 7,
            'temp_warn':     [(400, 100), (520, 100),(640, 100)],
            'temp_crit':     [(560, 80),  (0, 40)]  * 5,
            'apogee':        [(440, 110), (550, 110),(660, 110),(880, 110)],
            'high_attitude': [(750, 100), (0, 50)]  * 3,
            'mission_start': [(440, 90),  (550, 90), (660, 90),(770, 90),(880, 90)],
            'mission_stop':  [(880, 90),  (660, 90), (440, 90)],
        }

        @classmethod
        def _play_bg(cls, alarm_type):
            import platform
            if platform.system() == 'Windows':
                import winsound, time as _t
                beeps = cls._BEEP_PATTERNS.get(alarm_type, [(440, 150)])
                try:
                    for freq, ms in beeps:
                        if freq > 0:
                            winsound.Beep(freq, ms)
                        else:
                            _t.sleep(ms / 1000.0)
                except Exception:
                    pass
            else:
                import subprocess
                pcm = cls._build(alarm_type)
                try:
                    proc = subprocess.Popen(
                        ['aplay', '-r', str(cls.SAMPLE_RATE), '-f', 'S16_LE', '-c', '1', '-q', '-'],
                        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                    )
                    proc.stdin.write(pcm)
                    proc.stdin.close()
                    proc.wait(timeout=4)
                except Exception:
                    pass

        ALL_TYPES = [
            ('info',         '[INFO ] Generic info'),
            ('warn',         '[WARN ] Generic warning'),
            ('crit',         '[CRIT ] Critical alert'),
            ('lora_lost',    '[CRIT ] LoRa signal lost'),
            ('lora_ok',      '[INFO ] LoRa reconnected'),
            ('gps_lost',     '[WARN ] GPS signal lost'),
            ('gps_ok',       '[INFO ] GPS fix acquired'),
            ('batt_warn',    '[WARN ] Battery low'),
            ('batt_crit',    '[CRIT ] Battery critical'),
            ('temp_warn',    '[WARN ] High temperature'),
            ('temp_crit',    '[CRIT ] Temp critical'),
            ('apogee',       '[INFO ] Apogee detected'),
            ('high_attitude','[WARN ] High roll/pitch'),
            ('mission_start','[INFO ] Mission started'),
            ('mission_stop', '[INFO ] Mission stopped'),
        ]


    class BigValueWidget(QWidget):
        """Large prominent data display with accent bar."""
        def __init__(self, label, unit, color="#00d4ff", parent=None):
            super().__init__(parent)
            self.label = label
            self.unit = unit
            self.value_str = "--"
            self.accent = QColor(color)
            self.setMinimumSize(70, 55)

        def set_value(self, text, color=None):
            self.value_str = str(text)
            if color:
                self.accent = QColor(color)
            self.update()

        def paintEvent(self, event):
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            w, h = self.width(), self.height()

            p.fillRect(0, 0, w, h, QColor(3, 7, 14))
            p.fillRect(0, 0, 3, h, self.accent)
            p.setPen(QPen(QColor(self.accent.red(), self.accent.green(), self.accent.blue(), 35), 1))
            p.setBrush(Qt.NoBrush)
            p.drawRect(0, 0, w - 1, h - 1)

            p.setPen(QColor(55, 85, 105))
            p.setFont(QFont("Consolas", 9, QFont.Bold))
            p.drawText(7, 14, self.label)

            font_size = 16 if len(self.value_str) <= 7 else 12
            p.setFont(QFont("Consolas", font_size, QFont.Bold))
            p.setPen(self.accent)
            fm = p.fontMetrics()
            vw = fm.horizontalAdvance(self.value_str)
            p.drawText(max(7, (w - vw) // 2), h - 14, self.value_str)

            p.setFont(QFont("Consolas", 11, QFont.Bold))
            p.setPen(QColor(self.accent.red(), self.accent.green(), self.accent.blue(), 200))
            uw = p.fontMetrics().horizontalAdvance(self.unit)
            p.drawText(w - uw - 4, 14, self.unit)
            p.end()


    class SparklineWidget(QWidget):
        """Real-time sparkline chart with fill gradient."""
        def __init__(self, label, unit="", color="#00d4ff", maxpoints=120, parent=None):
            super().__init__(parent)
            self.label = label
            self.unit = unit
            self.accent = QColor(color)
            self.data = deque(maxlen=maxpoints)
            self.setMinimumSize(80, 55)

        def push(self, value):
            self.data.append(float(value))
            self.update()

        def paintEvent(self, event):
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            w, h = self.width(), self.height()
            r, g, b = self.accent.red(), self.accent.green(), self.accent.blue()

            p.fillRect(0, 0, w, h, QColor(2, 5, 10))

            # Grid
            p.setPen(QPen(QColor(12, 25, 38), 1))
            for i in range(1, 4):
                y = int(h * i / 4)
                p.drawLine(0, y, w, y)

            mt, mb = 18, 14
            ph = h - mt - mb

            p.setPen(QColor(45, 70, 90))
            p.setFont(QFont("Consolas", 8))
            p.drawText(5, 12, self.label)

            if len(self.data) >= 2:
                pts = list(self.data)
                n = len(pts)
                mn, mx = min(pts), max(pts)
                rng = mx - mn if mx != mn else 1.0

                def px(i, v):
                    return (i / (n - 1)) * w, mt + ph - ((v - mn) / rng) * ph

                path = QPainterPath()
                for i, v in enumerate(pts):
                    x, y = px(i, v)
                    if i == 0:
                        path.moveTo(x, y)
                    else:
                        path.lineTo(x, y)

                fill = QPainterPath(path)
                fill.lineTo(w, mt + ph)
                fill.lineTo(0, mt + ph)
                fill.closeSubpath()
                grad = QLinearGradient(0, mt, 0, mt + ph)
                grad.setColorAt(0, QColor(r, g, b, 65))
                grad.setColorAt(1, QColor(r, g, b, 4))
                p.setBrush(QBrush(grad))
                p.setPen(Qt.NoPen)
                p.drawPath(fill)

                p.setPen(QPen(self.accent, 1.5))
                p.setBrush(Qt.NoBrush)
                p.drawPath(path)

                cx, cy = px(n - 1, pts[-1])
                p.setBrush(self.accent)
                p.setPen(Qt.NoPen)
                p.drawEllipse(int(cx) - 3, int(cy) - 3, 6, 6)

                p.setPen(self.accent)
                p.setFont(QFont("Consolas", 9, QFont.Bold))
                val_txt = "%.1f%s" % (pts[-1], self.unit)
                p.drawText(w - p.fontMetrics().horizontalAdvance(val_txt) - 4, 12, val_txt)

                p.setPen(QColor(40, 65, 85))
                p.setFont(QFont("Consolas", 7))
                p.drawText(3, h - 2, "%.0f" % mn)
                p.drawText(3, mt + 9, "%.0f" % mx)
            else:
                p.setPen(QColor(30, 50, 70))
                p.setFont(QFont("Consolas", 9))
                p.drawText(w // 2 - 30, h // 2 + 5, "no data")
            p.end()


    class SpacebotGCS(QMainWindow):
        def __init__(self):
            super().__init__()
            self.serial = SerialHandler()

            self.mission_start_time = None
            self.max_altitude = 0.0
            self.prev_altitude = 0.0
            self.prev_alt_time = time.time()
            self.vertical_velocity = 0.0
            self.apogee_reached = False
            self.flight_state = 0
            self.log_enabled = False
            self.log_file = None

            # Alarm threshold configuration -- load dari file, fallback ke default
            self.load_config()

            # Alarm state tracking (detect transitions only)
            self._alm_lora_ok    = True
            self._alm_gps_valid  = False
            self._alm_batt_warn  = False
            self._alm_batt_crit  = False
            self._alm_temp_warn  = False
            self._alm_temp_crit  = False
            self._alm_pres_warn  = False
            self._alm_pres_crit  = False
            self._alm_pwr_warn   = False
            self._alm_pwr_crit   = False
            self._alm_att_warn      = False
            self.max_attitude_angle = 30

            # --- Uplink LOCK (GCS → wfb_tx -p1 → RF → RPi → K230 UDP:5601) ---
            # Kirim STATE berulang (bukan edge) → tahan paket hilang di link RF.
            self.lock_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.lock_addr = ("127.0.0.1", 5601)   # wfb_tx -p1 -u 5601 baca dari sini

            self.init_ui()
            self.connect_signals()
            self.update_timer = QTimer()
            self.update_timer.timeout.connect(self.update_display)
            self.update_timer.start(50)

            # Timer kirim status lock berkala (~5x/detik) ke uplink
            self.lock_timer = QTimer()
            self.lock_timer.timeout.connect(self.send_lock_state)
            self.lock_timer.start(200)
            
        def init_ui(self):
            self.setWindowTitle("SPACEBOT GCS v6.0 -- ORBITAL COMMAND")
            self.setMinimumSize(1280, 700)
            self.setStyleSheet("""
                QMainWindow { background-color: #03050a; }
                QWidget { background-color: transparent; }
                QLabel { color: #00ccff; font-family: 'Consolas'; }
                QSplitter::handle { background-color: #0d2137; width: 3px; height: 3px; }
                QGroupBox {
                    color: #00ccff;
                    border: 1px solid #0d3a5c;
                    border-radius: 6px;
                    margin-top: 10px;
                    padding-top: 4px;
                    font-family: 'Consolas';
                    font-weight: bold;
                    font-size: 10px;
                    background-color: #04080f;
                }
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 12px;
                    padding: 0 6px;
                    background-color: #03050a;
                    color: #00aadd;
                    letter-spacing: 2px;
                }
                QPushButton {
                    background-color: #050d1a;
                    color: #00ccff;
                    border: 1px solid #0d4a70;
                    border-radius: 4px;
                    padding: 4px 10px;
                    font-family: 'Consolas';
                    font-weight: bold;
                    font-size: 10px;
                }
                QPushButton:hover { background-color: #0a1f35; border-color: #00ccff; }
                QPushButton:pressed { background-color: #00334d; }
                QComboBox {
                    background-color: #050d1a;
                    color: #00ccff;
                    border: 1px solid #0d4a70;
                    border-radius: 4px;
                    padding: 3px 6px;
                    font-family: 'Consolas';
                }
                QComboBox::drop-down { border: none; width: 20px; }
                QComboBox QAbstractItemView {
                    background-color: #050d1a;
                    color: #00ccff;
                    selection-background-color: #0d4a70;
                    border: 1px solid #0d4a70;
                }
                QProgressBar {
                    border: 1px solid #0d4a70;
                    border-radius: 3px;
                    background-color: #050d1a;
                    text-align: center;
                    color: #00ccff;
                    font-family: 'Consolas';
                    font-size: 10px;
                }
                QProgressBar::chunk { background-color: #00aadd; border-radius: 2px; }
                QTextEdit {
                    background-color: #020408;
                    color: #00aacc;
                    border: 1px solid #0d2137;
                    font-family: 'Consolas';
                    font-size: 10px;
                }
                QCheckBox { color: #00aacc; font-family: 'Consolas'; font-size: 10px; }
                QCheckBox::indicator { border: 1px solid #0d4a70; background-color: #050d1a; width: 12px; height: 12px; }
                QCheckBox::indicator:checked { background-color: #00aadd; }
                QScrollBar:vertical { background: #030508; width: 8px; }
                QScrollBar::handle:vertical { background: #0d3a5c; border-radius: 4px; }
            """)

            central = QWidget()
            central.setStyleSheet("background-color: #03050a;")
            self.setCentralWidget(central)
            main_layout = QVBoxLayout(central)
            main_layout.setSpacing(4)
            main_layout.setContentsMargins(5, 5, 5, 5)

            # ?? HEADER BAR ??????????????????????????????????????????????????
            header = QFrame()
            header.setFixedHeight(32)
            header.setStyleSheet("""
                QFrame {
                    background-color: #04080f;
                    border: 1px solid #0d3a5c;
                    border-radius: 4px;
                }
            """)
            h_layout = QHBoxLayout(header)
            h_layout.setContentsMargins(12, 4, 12, 4)
            h_layout.setSpacing(6)

            title_label = QLabel("[*]  SPACEBOT GCS  v6.0")
            title_label.setStyleSheet(
                "color: #00ccff; font-size: 13px; font-weight: bold; "
                "font-family: 'Consolas'; letter-spacing: 3px;"
            )
            h_layout.addWidget(title_label)
            h_layout.addSpacing(16)

            sep = QLabel("|")
            sep.setStyleSheet("color: #0d3a5c; font-size: 20px;")
            h_layout.addWidget(sep)

            self.port_combo = QComboBox()
            self.port_combo.setFixedWidth(90)
            self.refresh_ports()
            h_layout.addWidget(self.port_combo)

            self.refresh_btn = QPushButton("R")
            self.refresh_btn.setFixedWidth(32)
            self.refresh_btn.setToolTip("Refresh Ports")
            self.refresh_btn.clicked.connect(self.refresh_ports)
            h_layout.addWidget(self.refresh_btn)

            self.connect_btn = QPushButton("CONNECT")
            self.connect_btn.setMinimumWidth(90)
            self.connect_btn.clicked.connect(self.toggle_connection)
            h_layout.addWidget(self.connect_btn)

            h_layout.addSpacing(8)
            self.status_label = QLabel("[O]  OFFLINE")
            self.status_label.setStyleSheet(
                "color: #ff3333; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
            )
            h_layout.addWidget(self.status_label)

            h_layout.addStretch()

            self.alarm_bar = QLabel("  NO ALARMS  ")
            self.alarm_bar.setAlignment(Qt.AlignCenter)
            self.alarm_bar.setFixedHeight(22)
            self.alarm_bar.setFixedWidth(120)
            self.alarm_bar.setStyleSheet(
                "color: #005533; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                "background-color: #001a0a; border: 1px solid #003322; border-radius: 3px; padding: 0 6px;"
            )
            h_layout.addWidget(self.alarm_bar)
            h_layout.addSpacing(16)

            # Mission stats in header
            self.mission_timer_label = QLabel("T+  --:--:--")
            self.mission_timer_label.setStyleSheet(
                "color: #7b2fff; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
            )
            h_layout.addWidget(self.mission_timer_label)

            h_layout.addSpacing(20)
            self.maxalt_label = QLabel("MAX ALT: --.- m")
            self.maxalt_label.setStyleSheet(
                "color: #ff9900; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
            )
            h_layout.addWidget(self.maxalt_label)

            h_layout.addSpacing(20)
            self.vvel_header_label = QLabel("V-VEL: -- m/s")
            self.vvel_header_label.setStyleSheet(
                "color: #00ffaa; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
            )
            h_layout.addWidget(self.vvel_header_label)

            h_layout.addSpacing(20)
            self.packet_label = QLabel("PKT: 0")
            self.packet_label.setStyleSheet(
                "color: #005577; font-family: 'Consolas'; font-size: 10px;"
            )
            h_layout.addWidget(self.packet_label)

            h_layout.addSpacing(10)
            self.pc_batt_label = QLabel("PC [--]")
            self.pc_batt_label.setAlignment(Qt.AlignCenter)
            self.pc_batt_label.setFixedHeight(22)
            self.pc_batt_label.setMinimumWidth(80)
            self.pc_batt_label.setStyleSheet(
                "color: #445566; font-family: 'Consolas'; font-size: 10px; font-weight: bold;"
                "background-color: #040810; border: 1px solid #0a1e2e; border-radius: 3px; padding: 0 6px;"
            )
            if not PSUTIL_AVAILABLE:
                self.pc_batt_label.setText("PC [N/A]")
                self.pc_batt_label.setToolTip("Install psutil: pip install psutil")
            h_layout.addWidget(self.pc_batt_label)

            h_layout.addSpacing(6)
            self.config_btn = QPushButton("[*] CONFIG")
            self.config_btn.clicked.connect(self.open_config_dialog)
            self.config_btn.setStyleSheet(
                "QPushButton { background-color: #050d1a; color: #ffaa00; border: 1px solid #664400; }"
                "QPushButton:hover { background-color: #1a1000; border-color: #ffaa00; }"
            )
            h_layout.addWidget(self.config_btn)

            h_layout.addSpacing(6)
            self.exit_btn = QPushButton("X EXIT")
            self.exit_btn.clicked.connect(self.close)
            self.exit_btn.setStyleSheet(
                "QPushButton { background-color: #1a0000; color: #ff3333; border: 1px solid #660000; }"
                "QPushButton:hover { background-color: #330000; border-color: #ff3333; }"
            )
            h_layout.addWidget(self.exit_btn)

            main_layout.addWidget(header)

            # ?? MAIN CONTENT: splitter LEFT | CENTER(video) | RIGHT ?????????
            splitter = QSplitter(Qt.Horizontal)
            splitter.setChildrenCollapsible(False)
            splitter.setStyleSheet("QSplitter::handle { background-color: #0d2137; }")

            # LEFT PANEL -- Control + Sensor
            left_w = QWidget()
            left_w.setStyleSheet("background-color: transparent;")
            left_lay = QVBoxLayout(left_w)
            left_lay.setSpacing(6)
            left_lay.setContentsMargins(0, 0, 0, 0)
            left_lay.addWidget(self.create_control_panel())
            left_lay.addWidget(self.create_sensor_panel())
            splitter.addWidget(left_w)

            # CENTER PANEL -- Video (dominant)
            splitter.addWidget(self.create_video_panel())

            # RIGHT PANEL -- Telemetry + Map
            right_w = QWidget()
            right_w.setStyleSheet("background-color: transparent;")
            right_lay = QVBoxLayout(right_w)
            right_lay.setSpacing(6)
            right_lay.setContentsMargins(0, 0, 0, 0)
            right_lay.addWidget(self.create_telemetry_panel(), 2)
            right_lay.addWidget(self.create_map_panel(), 3)
            splitter.addWidget(right_w)

            splitter.setSizes([230, 680, 290])
            main_layout.addWidget(splitter, 1)

            # ?? FLIGHT STATE MACHINE ??????????????????????????????????????????
            main_layout.addWidget(self.create_flight_state_panel())

            # ?? BOTTOM: Raw Data ????????????????????????????????????????????
            raw_group = self.create_raw_data_panel()
            raw_group.setMaximumHeight(140)
            main_layout.addWidget(raw_group)
            
        def create_control_panel(self):
            group = QGroupBox("CONTROL DATA")
            layout = QVBoxLayout(group)
            
            joy_layout = QHBoxLayout()
            self.joy1_widget = JoystickWidget("JOY 1 (Throttle)")
            self.joy2_widget = JoystickWidget("JOY 2 (Direction)")
            joy_layout.addWidget(self.joy1_widget)
            joy_layout.addWidget(self.joy2_widget)
            layout.addLayout(joy_layout)
            
            sw_frame = QFrame()
            sw_frame.setStyleSheet("QFrame { background-color: #0f0f0f; border-radius: 5px; }")
            sw_layout = QHBoxLayout(sw_frame)
            sw_layout.setContentsMargins(10, 10, 10, 10)
            sw_layout.addWidget(QLabel("SWITCHES:"))
            
            self.sw_indicators = []
            for i in range(5):
                indicator = QLabel("SW%d" % (i+1))
                indicator.setAlignment(Qt.AlignCenter)
                indicator.setFixedSize(50, 30)
                indicator.setStyleSheet("background-color: #1a1a1a; border: 1px solid #333333; border-radius: 5px; color: #444444; font-family: 'Consolas'; font-weight: bold;")
                self.sw_indicators.append(indicator)
                sw_layout.addWidget(indicator)
            sw_layout.addStretch()
            layout.addWidget(sw_frame)

            # BTN_MERAH & BTN_HIJAU -- baris terpisah agar lebar panel tidak berubah
            btn_frame = QFrame()
            btn_frame.setStyleSheet("QFrame { background-color: #0f0f0f; border-radius: 5px; }")
            btn_layout = QHBoxLayout(btn_frame)
            btn_layout.setContentsMargins(10, 6, 10, 6)
            btn_layout.setSpacing(8)

            btn_lbl = QLabel("BUTTONS:")
            btn_lbl.setStyleSheet("color: #00ccff; font-family: 'Consolas'; font-size: 9px;")
            btn_layout.addWidget(btn_lbl)

            self.btn_merah_indicator = QLabel("BTN MERAH")
            self.btn_merah_indicator.setAlignment(Qt.AlignCenter)
            self.btn_merah_indicator.setFixedHeight(26)
            self.btn_merah_indicator.setStyleSheet(
                "background-color: #1a0000; border: 1px solid #440000; border-radius: 5px; "
                "color: #552222; font-family: 'Consolas'; font-weight: bold; font-size: 9px;"
            )
            btn_layout.addWidget(self.btn_merah_indicator, 1)

            self.btn_hijau_indicator = QLabel("BTN HIJAU")
            self.btn_hijau_indicator.setAlignment(Qt.AlignCenter)
            self.btn_hijau_indicator.setFixedHeight(26)
            self.btn_hijau_indicator.setStyleSheet(
                "background-color: #001a00; border: 1px solid #004400; border-radius: 5px; "
                "color: #225522; font-family: 'Consolas'; font-weight: bold; font-size: 9px;"
            )
            btn_layout.addWidget(self.btn_hijau_indicator, 1)
            layout.addWidget(btn_frame)

            nrf_frame = QFrame()
            nrf_frame.setStyleSheet("QFrame { background-color: #0f0f0f; border-radius: 5px; }")
            nrf_layout = QHBoxLayout(nrf_frame)
            nrf_layout.setContentsMargins(10, 5, 10, 5)
            nrf_layout.addWidget(QLabel("NRF24:"))

            self.nrf_rate_bar = QProgressBar()
            self.nrf_rate_bar.setRange(0, 100)
            self.nrf_rate_bar.setFormat("%v%")
            self.nrf_rate_bar.setFixedHeight(16)
            nrf_layout.addWidget(self.nrf_rate_bar)

            self.nrf_rssi_label = QLabel("-110dBm")
            self.nrf_rssi_label.setStyleSheet(
                "color: #888888; font-family: 'Consolas'; font-size: 9px; min-width: 58px;"
            )
            nrf_layout.addWidget(self.nrf_rssi_label)

            self.nrf_count_label = QLabel("TX:0 OK:0")
            self.nrf_count_label.setStyleSheet("font-family: 'Consolas'; font-size: 9px;")
            nrf_layout.addWidget(self.nrf_count_label)

            self.nrf_conn_indicator = QLabel("NO ACK")
            self.nrf_conn_indicator.setAlignment(Qt.AlignCenter)
            self.nrf_conn_indicator.setFixedSize(58, 20)
            self.nrf_conn_indicator.setStyleSheet(
                "background-color: #1a0000; border: 1px solid #440000; border-radius: 3px; "
                "color: #441111; font-family: 'Consolas'; font-size: 9px; font-weight: bold;"
            )
            nrf_layout.addWidget(self.nrf_conn_indicator)
            layout.addWidget(nrf_frame)
            
            return group
            
        def create_telemetry_panel(self):
            group = QGroupBox("TELEMETRY -- LoRa")
            layout = QVBoxLayout(group)
            layout.setSpacing(4)
            layout.setContentsMargins(3, 10, 3, 3)

            # LoRa status bar
            self.lora_status = QLabel("[O] LoRa: OFFLINE")
            self.lora_status.setAlignment(Qt.AlignCenter)
            self.lora_status.setFixedHeight(20)
            self.lora_status.setStyleSheet(
                "color: #ff3333; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                "background-color: #130000; border: 1px solid #440000; border-radius: 3px; padding: 2px;"
            )
            layout.addWidget(self.lora_status)

            # Row 1: Flight data
            r1 = QHBoxLayout()
            r1.setSpacing(3)
            self.bv_altitude = BigValueWidget("ALTITUDE", "m",   "#00ff88")
            self.bv_vvel     = BigValueWidget("V-VELOCITY","m/s","#00d4ff")
            self.bv_maxalt   = BigValueWidget("MAX ALT",  "m",   "#ff9900")
            for w in (self.bv_altitude, self.bv_vvel, self.bv_maxalt):
                r1.addWidget(w)
            layout.addLayout(r1)

            # Row 2: Power
            r2 = QHBoxLayout()
            r2.setSpacing(3)
            self.bv_voltage = BigValueWidget("VOLTAGE", "V",  "#ffdd00")
            self.bv_current = BigValueWidget("CURRENT", "A",  "#00ccff")
            self.bv_batt    = BigValueWidget("BATTERY", "%",  "#ffaa00")
            for w in (self.bv_voltage, self.bv_current, self.bv_batt):
                r2.addWidget(w)
            layout.addLayout(r2)

            # Row 3: Environment
            r3 = QHBoxLayout()
            r3.setSpacing(3)
            self.bv_temp     = BigValueWidget("TEMP",     "C",   "#ff6600")
            self.bv_pressure = BigValueWidget("PRESSURE", "hPa", "#88ff88")
            self.bv_power    = BigValueWidget("POWER",    "W",   "#ff44ff")
            for w in (self.bv_temp, self.bv_pressure, self.bv_power):
                r3.addWidget(w)
            layout.addLayout(r3)

            # Apogee banner
            self.apogee_label = QLabel("APOGEE: --")
            self.apogee_label.setAlignment(Qt.AlignCenter)
            self.apogee_label.setFixedHeight(18)
            self.apogee_label.setStyleSheet(
                "color: #222233; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                "background-color: #04040a; border-radius: 3px;"
            )
            layout.addWidget(self.apogee_label)

            # proxy labels (not shown, keep update_display compat)
            self.altitude_label = QLabel()
            self.vvel_label     = QLabel()
            self.batt_label     = QLabel()
            self.voltage_label  = QLabel()
            self.current_label  = QLabel()
            self.power_label    = QLabel()
            self.temp_label     = QLabel()
            self.pressure_label = QLabel()

            return group
            
        def create_map_panel(self):
            group = QGroupBox("MAP [Real GPS + Compass]")
            layout = QVBoxLayout(group)
            
            self.map_widget = OfflineMapWidget()
            layout.addWidget(self.map_widget)
            
            ctrl_frame = QFrame()
            ctrl_frame.setStyleSheet("QFrame { background-color: #0f0f0f; border-radius: 5px; }")
            ctrl_layout = QHBoxLayout(ctrl_frame)
            ctrl_layout.setContentsMargins(5, 5, 5, 5)
            
            self.btn_auto = QPushButton("AUTO")
            self.btn_auto.setCheckable(True)
            self.btn_auto.setChecked(True)
            self.btn_auto.clicked.connect(self.toggle_auto_center)
            self.btn_auto.setStyleSheet("QPushButton:checked { background-color: #004400; border-color: #00ff00; color: #00ff00; }")
            ctrl_layout.addWidget(self.btn_auto)
            
            btn_zoom_in = QPushButton("+")
            btn_zoom_in.setFixedWidth(30)
            btn_zoom_in.clicked.connect(self.map_widget.zoom_in)
            ctrl_layout.addWidget(btn_zoom_in)
            
            btn_zoom_out = QPushButton("-")
            btn_zoom_out.setFixedWidth(30)
            btn_zoom_out.clicked.connect(self.map_widget.zoom_out)
            ctrl_layout.addWidget(btn_zoom_out)
            
            btn_center = QPushButton("CENTER")
            btn_center.clicked.connect(self.map_widget.center_on_drone)
            ctrl_layout.addWidget(btn_center)
            
            btn_home = QPushButton("HOME")
            btn_home.clicked.connect(self.map_widget.center_on_home)
            ctrl_layout.addWidget(btn_home)
            
            btn_clear = QPushButton("CLR TRAIL")
            btn_clear.clicked.connect(self.map_widget.clear_trail)
            ctrl_layout.addWidget(btn_clear)
            
            ctrl_layout.addStretch()
            
            self.gps_label = QLabel("GPS: N/A")
            self.gps_label.setStyleSheet("color: #888888; font-family: 'Consolas'; font-size: 10px;")
            ctrl_layout.addWidget(self.gps_label)
            
            layout.addWidget(ctrl_frame)
            return group
        
        def toggle_auto_center(self):
            self.map_widget.toggle_auto_center()
            self.btn_auto.setChecked(self.map_widget.auto_center)

        def load_config(self):
            cfg = dict(_CFG_DEFAULTS)
            try:
                with open(CONFIG_FILE, 'r') as f:
                    saved = json.load(f)
                cfg.update({k: v for k, v in saved.items() if k in cfg})
            except Exception:
                pass
            self.cfg_temp_warn = float(cfg["temp_warn"])
            self.cfg_temp_crit = float(cfg["temp_crit"])
            self.cfg_volt_min  = float(cfg["volt_min"])
            self.cfg_volt_max  = float(cfg["volt_max"])
            self.cfg_volt_warn = int(cfg["volt_warn"])
            self.cfg_volt_crit = int(cfg["volt_crit"])
            self.cfg_pres_min = float(cfg["pres_min"])
            self.cfg_pres_max = float(cfg["pres_max"])
            self.cfg_pwr_warn = float(cfg["pwr_warn"])
            self.cfg_pwr_crit = float(cfg["pwr_crit"])

            # servo config per channel (loaded separately from "servo" key)
            self.cfg_servo = [dict(_SERVO_CH_DEFAULT) for _ in range(SERVO_NUM_CH)]
            try:
                with open(CONFIG_FILE, 'r') as f2:
                    saved_all = json.load(f2)
                servo_list = saved_all.get("servo", [])
                if isinstance(servo_list, list):
                    for i, ch in enumerate(servo_list[:SERVO_NUM_CH]):
                        if isinstance(ch, dict):
                            self.cfg_servo[i]["trim"] = int(ch.get("trim", 0))
                            self.cfg_servo[i]["epaL"] = int(ch.get("epaL", 100))
                            self.cfg_servo[i]["epaR"] = int(ch.get("epaR", 100))
            except Exception:
                pass

        def save_config(self):
            data = {
                "temp_warn": self.cfg_temp_warn,
                "temp_crit": self.cfg_temp_crit,
                "volt_min":  self.cfg_volt_min,
                "volt_max":  self.cfg_volt_max,
                "volt_warn": self.cfg_volt_warn,
                "volt_crit": self.cfg_volt_crit,
                "pres_min":  self.cfg_pres_min,
                "pres_max":  self.cfg_pres_max,
                "pwr_warn":  self.cfg_pwr_warn,
                "pwr_crit":  self.cfg_pwr_crit,
                "servo":     self.cfg_servo,
            }
            try:
                with open(CONFIG_FILE, 'w') as f:
                    json.dump(data, f, indent=2)
            except Exception:
                pass

        def open_config_dialog(self):
            DLG_STYLE = """
                QDialog  { background-color: #03050a; border: 1px solid #0d3a5c; }
                QLabel   { color: #00aadd; font-family: 'Consolas'; font-size: 10px; }
                QLineEdit {
                    background-color: #050d1a; color: #00ccff;
                    border: 1px solid #0d4a70; border-radius: 3px;
                    padding: 3px 6px; font-family: 'Consolas'; font-size: 10px;
                }
                QPushButton {
                    background-color: #050d1a; color: #00ccff;
                    border: 1px solid #0d4a70; border-radius: 3px;
                    padding: 5px 12px; font-family: 'Consolas'; font-size: 10px;
                }
                QPushButton:hover { background-color: #0a1f35; border-color: #00ccff; }
                QFrame#section { border: 1px solid #0d3a5c; border-radius: 4px; }
            """

            dlg = QDialog(self)
            dlg.setWindowTitle("ALARM LIMITS CONFIGURATION")
            dlg.setFixedWidth(400)
            dlg.setStyleSheet(DLG_STYLE)

            lay = QVBoxLayout(dlg)
            lay.setSpacing(10)
            lay.setContentsMargins(16, 16, 16, 16)

            title = QLabel("ALARM LIMIT SETTINGS")
            title.setStyleSheet(
                "color:#ffaa00; font-weight:bold; font-size:13px; font-family:'Consolas';"
                "border-bottom: 1px solid #664400; padding-bottom: 4px;"
            )
            title.setAlignment(Qt.AlignCenter)
            lay.addWidget(title)

            def _section(label):
                lbl = QLabel("  " + label)
                lbl.setStyleSheet(
                    "color:#005577; font-family:'Consolas'; font-size:9px; font-weight:bold;"
                    "letter-spacing:2px; background-color:#020508; padding:2px 0;"
                )
                return lbl

            def _row(grid, row, label, field):
                lbl = QLabel(label)
                lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                grid.addWidget(lbl, row, 0)
                grid.addWidget(field, row, 1)

            def _input(val):
                f = QLineEdit(str(val))
                f.setFixedWidth(90)
                return f

            # --- TEMPERATURE ---
            lay.addWidget(_section("TEMPERATURE"))
            temp_grid = QGridLayout()
            temp_grid.setSpacing(5)
            temp_grid.setColumnMinimumWidth(0, 160)
            f_temp_warn = _input(self.cfg_temp_warn)
            f_temp_crit = _input(self.cfg_temp_crit)
            _row(temp_grid, 0, "Warn threshold (degC):", f_temp_warn)
            _row(temp_grid, 1, "Crit threshold (degC):", f_temp_crit)
            lay.addLayout(temp_grid)

            # --- VOLTAGE / BATTERY ---
            lay.addWidget(_section("VOLTAGE / BATTERY"))
            volt_grid = QGridLayout()
            volt_grid.setSpacing(5)
            volt_grid.setColumnMinimumWidth(0, 160)
            f_volt_min  = _input(self.cfg_volt_min)
            f_volt_max  = _input(self.cfg_volt_max)
            f_volt_warn = _input(self.cfg_volt_warn)
            f_volt_crit = _input(self.cfg_volt_crit)
            _row(volt_grid, 0, "Min voltage (V) = 0%:",   f_volt_min)
            _row(volt_grid, 1, "Max voltage (V) = 100%:", f_volt_max)
            _row(volt_grid, 2, "Warn level (%):",          f_volt_warn)
            _row(volt_grid, 3, "Crit level (%):",          f_volt_crit)
            lay.addLayout(volt_grid)

            # --- PRESSURE ---
            lay.addWidget(_section("PRESSURE  (0 = disabled)"))
            pres_grid = QGridLayout()
            pres_grid.setSpacing(5)
            pres_grid.setColumnMinimumWidth(0, 160)
            f_pres_min = _input(self.cfg_pres_min)
            f_pres_max = _input(self.cfg_pres_max)
            _row(pres_grid, 0, "Min normal (hPa):", f_pres_min)
            _row(pres_grid, 1, "Max normal (hPa):", f_pres_max)
            lay.addLayout(pres_grid)

            # --- POWER ---
            lay.addWidget(_section("POWER  (0 = disabled)"))
            pwr_grid = QGridLayout()
            pwr_grid.setSpacing(5)
            pwr_grid.setColumnMinimumWidth(0, 160)
            f_pwr_warn = _input(self.cfg_pwr_warn)
            f_pwr_crit = _input(self.cfg_pwr_crit)
            _row(pwr_grid, 0, "Warn threshold (W):", f_pwr_warn)
            _row(pwr_grid, 1, "Crit threshold (W):", f_pwr_crit)
            lay.addLayout(pwr_grid)

            # --- SERVO CONFIG (per channel PCA9685 0-8) ---
            lay.addWidget(_section("SERVO CONFIG  (per channel PCA9685, via NRF24)"))

            # Channel selector row
            ch_row = QHBoxLayout()
            ch_lbl = QLabel("Channel PCA9685:")
            ch_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            ch_combo = QComboBox()
            ch_combo.setFixedWidth(90)
            ch_combo.setStyleSheet(
                "QComboBox { background:#050d1a; color:#00ccff; border:1px solid #0d4a70;"
                "  border-radius:3px; padding:2px 6px; font-family:'Consolas'; font-size:10px; }"
                "QComboBox::drop-down { border:none; width:16px; }"
                "QComboBox QAbstractItemView { background:#050d1a; color:#00ccff; }"
            )
            for i in range(SERVO_NUM_CH):
                ch_combo.addItem("Ch %d" % i)
            ch_row.addWidget(ch_lbl)
            ch_row.addWidget(ch_combo)
            ch_row.addStretch()
            lay.addLayout(ch_row)

            servo_grid = QGridLayout()
            servo_grid.setSpacing(5)
            servo_grid.setColumnMinimumWidth(0, 160)
            f_servo_trim = _input(self.cfg_servo[0]["trim"])
            f_servo_epaL = _input(self.cfg_servo[0]["epaL"])
            f_servo_epaR = _input(self.cfg_servo[0]["epaR"])
            _row(servo_grid, 0, "Sub-Trim (-225..+225):", f_servo_trim)
            _row(servo_grid, 1, "EPA Left  (0-100%):",    f_servo_epaL)
            _row(servo_grid, 2, "EPA Right (0-100%):",    f_servo_epaR)
            lay.addLayout(servo_grid)

            servo_status_lbl = QLabel("")
            servo_status_lbl.setAlignment(Qt.AlignCenter)
            servo_status_lbl.setStyleSheet("color:#ffaa00; font-family:'Consolas'; font-size:9px;")

            def _on_ch_changed(idx):
                f_servo_trim.setText(str(self.cfg_servo[idx]["trim"]))
                f_servo_epaL.setText(str(self.cfg_servo[idx]["epaL"]))
                f_servo_epaR.setText(str(self.cfg_servo[idx]["epaR"]))
                servo_status_lbl.setText("")

            ch_combo.currentIndexChanged.connect(_on_ch_changed)

            def _send_servo():
                try:
                    ch = ch_combo.currentIndex()
                    st = int(f_servo_trim.text())
                    el = int(f_servo_epaL.text())
                    er = int(f_servo_epaR.text())
                    if not (-225 <= st <= 225):
                        raise ValueError("Sub-Trim harus -225 s/d +225")
                    if not (0 <= el <= 100):
                        raise ValueError("EPA Left harus 0-100")
                    if not (0 <= er <= 100):
                        raise ValueError("EPA Right harus 0-100")
                except ValueError as e:
                    servo_status_lbl.setText("ERR: %s" % str(e))
                    servo_status_lbl.setStyleSheet("color:#ff4444; font-family:'Consolas'; font-size:9px;")
                    return

                self.cfg_servo[ch]["trim"] = st
                self.cfg_servo[ch]["epaL"] = el
                self.cfg_servo[ch]["epaR"] = er
                self.save_config()

                cmd = "$SCFG|%d|%d|%d|%d" % (ch, st, el, er)
                if self.serial.send(cmd):
                    servo_status_lbl.setText("Ch%d sent: trim=%d epaL=%d%% epaR=%d%%" % (ch, st, el, er))
                    servo_status_lbl.setStyleSheet("color:#00ff88; font-family:'Consolas'; font-size:9px;")
                    self.log_event('INFO', 'Servo Ch%d: SubTrim=%d EpaL=%d%% EpaR=%d%%' % (ch, st, el, er))
                else:
                    servo_status_lbl.setText("GAGAL: serial tidak terhubung!")
                    servo_status_lbl.setStyleSheet("color:#ff4444; font-family:'Consolas'; font-size:9px;")

            send_servo_btn = QPushButton("SEND TO ROBOT")
            send_servo_btn.setStyleSheet(
                "QPushButton { color:#00ccff; border-color:#0066aa; background:#001020; }"
                "QPushButton:hover { background:#002040; border-color:#00ccff; }"
            )
            send_servo_btn.clicked.connect(_send_servo)

            lay.addWidget(send_servo_btn)
            lay.addWidget(servo_status_lbl)

            # Status
            status_lbl = QLabel("")
            status_lbl.setAlignment(Qt.AlignCenter)
            status_lbl.setStyleSheet("color:#ffaa00; font-family:'Consolas'; font-size:9px;")
            lay.addWidget(status_lbl)

            # Buttons
            btn_row = QHBoxLayout()
            btn_row.setSpacing(8)

            apply_btn = QPushButton("APPLY")
            apply_btn.setStyleSheet(
                "QPushButton { color:#00ff88; border-color:#00aa55; background:#001a0a; }"
                "QPushButton:hover { background:#002a10; }"
            )

            def _apply():
                try:
                    tw  = float(f_temp_warn.text())
                    tc  = float(f_temp_crit.text())
                    vmin = float(f_volt_min.text())
                    vmax = float(f_volt_max.text())
                    vw  = int(f_volt_warn.text())
                    vc  = int(f_volt_crit.text())
                    pm_min = float(f_pres_min.text())
                    pm_max = float(f_pres_max.text())
                    pw  = float(f_pwr_warn.text())
                    pc  = float(f_pwr_crit.text())

                    if tw >= tc:
                        raise ValueError("Temp warn harus < crit")
                    if vmin >= vmax:
                        raise ValueError("Volt min harus < max")
                    if vw <= vc:
                        raise ValueError("Volt warn % harus > crit %")
                    if pc > 0 and pw >= pc:
                        raise ValueError("Power warn harus < crit")
                except ValueError as e:
                    status_lbl.setText("ERR: %s" % str(e))
                    status_lbl.setStyleSheet("color:#ff4444; font-family:'Consolas'; font-size:9px;")
                    return

                self.cfg_temp_warn  = tw
                self.cfg_temp_crit  = tc
                self.cfg_volt_min   = vmin
                self.cfg_volt_max   = vmax
                self.cfg_volt_warn  = vw
                self.cfg_volt_crit  = vc
                self.cfg_pres_min   = pm_min
                self.cfg_pres_max   = pm_max
                self.cfg_pwr_warn   = pw
                self.cfg_pwr_crit   = pc

                # Reset alarm states agar langsung re-evaluate
                self._alm_batt_warn = False
                self._alm_batt_crit = False
                self._alm_temp_warn = False
                self._alm_temp_crit = False
                self._alm_pres_warn = False
                self._alm_pres_crit = False
                self._alm_pwr_warn  = False
                self._alm_pwr_crit  = False
                AlarmSystem.loop_stop('batt_crit')
                AlarmSystem.loop_stop('temp_crit')
                AlarmSystem.loop_stop('pwr_crit')

                self.save_config()
                self.log_event('INFO',
                    'Limits updated: T%.0f/%.0fC  V%.1f-%.1fV(warn%d%%/crit%d%%)  '
                    'P%.0f-%.0fhPa  Pwr%.0f/%.0fW' % (
                        tw, tc, vmin, vmax, vw, vc, pm_min, pm_max, pw, pc))
                status_lbl.setText("Saved to file.")
                status_lbl.setStyleSheet("color:#00ff88; font-family:'Consolas'; font-size:9px;")

            apply_btn.clicked.connect(_apply)
            btn_row.addWidget(apply_btn)

            def _reset():
                d = _CFG_DEFAULTS
                f_temp_warn.setText(str(d["temp_warn"]))
                f_temp_crit.setText(str(d["temp_crit"]))
                f_volt_min.setText(str(d["volt_min"]))
                f_volt_max.setText(str(d["volt_max"]))
                f_volt_warn.setText(str(d["volt_warn"]))
                f_volt_crit.setText(str(d["volt_crit"]))
                f_pres_min.setText(str(d["pres_min"]))
                f_pres_max.setText(str(d["pres_max"]))
                f_pwr_warn.setText(str(d["pwr_warn"]))
                f_pwr_crit.setText(str(d["pwr_crit"]))
                status_lbl.setText("Fields reset -- tekan APPLY untuk menyimpan.")
                status_lbl.setStyleSheet("color:#ffaa00; font-family:'Consolas'; font-size:9px;")

            reset_btn = QPushButton("DEFAULTS")
            reset_btn.setStyleSheet(
                "QPushButton { color:#ffaa00; border-color:#664400; background:#100800; }"
                "QPushButton:hover { background:#1a1000; }"
            )
            reset_btn.clicked.connect(_reset)
            btn_row.addWidget(reset_btn)

            close_btn = QPushButton("CLOSE")
            close_btn.setStyleSheet("color:#ff4444; border-color:#440000;")
            close_btn.clicked.connect(dlg.close)
            btn_row.addWidget(close_btn)

            lay.addLayout(btn_row)
            dlg.exec_()

        def toggle_mission(self):
            if self.mission_start_time is None:
                self.mission_start_time = time.time()
                self.max_altitude = 0.0
                self.apogee_reached = False
                self._alm_apogee_fired = False
                self.flight_state = 0
                self._refresh_state_labels()
                self.mission_btn.setText("[S] STOP MISSION")
                self.mission_btn.setStyleSheet(
                    "QPushButton { color:#ff4444; border-color:#aa2222; background:#1a0000; font-size:10px; }"
                )
                self.log_event('INFO', 'Mission STARTED', 'mission_start')
            else:
                elapsed = time.time() - self.mission_start_time
                self.mission_start_time = None
                self.mission_btn.setText("> START MISSION")
                self.mission_btn.setStyleSheet(
                    "QPushButton { color:#00ff88; border-color:#00aa55; background:#001a0a; font-size:10px; }"
                )
                self.mission_timer_label.setText("T+  --:--:--")
                self.log_event('INFO', 'Mission STOPPED -- duration %.0fs, max alt %.1fm' % (elapsed, self.max_altitude), 'mission_stop')

        def log_event(self, level, message, alarm_type=None):
            """Append colored entry to event log and play alarm."""
            ts = datetime.now().strftime("%H:%M:%S")
            if level == 'INFO':
                badge = '<span style="color:#00aadd;font-weight:bold;">[INFO ]</span>'
            elif level == 'WARN':
                badge = '<span style="color:#ff9900;font-weight:bold;">[WARN ]</span>'
            elif level == 'CRIT':
                badge = '<span style="color:#ff3333;font-weight:bold;">[CRIT ]</span>'
            else:
                badge = '<span style="color:#888888;">[    ]</span>'

            ts_html   = '<span style="color:#1e4a60;">%s</span>' % ts
            msg_html  = '<span style="color:#aabbcc;">%s</span>' % message
            line = '%s %s %s' % (ts_html, badge, msg_html)
            self.event_log.append(line)
            sb = self.event_log.verticalScrollBar()
            sb.setValue(sb.maximum())

            if alarm_type:
                AlarmSystem.play(alarm_type)

        def open_test_dialog(self):
            dlg = QDialog(self)
            dlg.setWindowTitle("ALARM TEST PANEL")
            dlg.setStyleSheet("""
                QDialog { background-color: #03050a; border: 1px solid #0d3a5c; }
                QLabel  { color: #00aadd; font-family: 'Consolas'; font-size: 10px; }
                QPushButton {
                    background-color: #050d1a; color: #00ccff;
                    border: 1px solid #0d4a70; border-radius: 3px;
                    padding: 5px 10px; font-family: 'Consolas'; font-size: 10px;
                }
                QPushButton:hover { background-color: #0a1f35; }
            """)
            dlg.setMinimumWidth(380)
            lay = QVBoxLayout(dlg)
            lay.setSpacing(5)
            lay.setContentsMargins(12, 12, 12, 12)

            title = QLabel("TEST EACH ALARM SOUND")
            title.setStyleSheet("color:#00ccff; font-weight:bold; font-size:13px; font-family:'Consolas';")
            lay.addWidget(title)

            for atype, desc in AlarmSystem.ALL_TYPES:
                row = QHBoxLayout()
                lbl = QLabel(desc)
                lbl.setMinimumWidth(240)
                btn = QPushButton("PLAY")
                btn.setFixedWidth(60)
                btn.clicked.connect(lambda _, a=atype: AlarmSystem.play(a))
                row.addWidget(lbl)
                row.addWidget(btn)
                lay.addLayout(row)

            close_btn = QPushButton("CLOSE")
            close_btn.clicked.connect(dlg.close)
            close_btn.setStyleSheet("color:#ff4444; border-color:#440000;")
            lay.addWidget(close_btn)
            dlg.exec_()

        def toggle_logging(self):
            if not self.log_enabled:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = "gcs_log_%s.csv" % ts
                try:
                    self.log_file = open(filename, 'w')
                    self.log_file.write(
                        "timestamp,roll,pitch,yaw,voltage,current,power,"
                        "pressure,altitude,temperature,lat,lon,speed,sats,heading\n"
                    )
                    self.log_enabled = True
                    self.log_btn.setChecked(True)
                    self.log_status_label.setText("Log: %s" % filename)
                    self.log_status_label.setStyleSheet("color: #ff4444; font-size: 10px;")
                except Exception as e:
                    self.log_btn.setChecked(False)
                    self.log_status_label.setText("Log ERR: %s" % str(e))
            else:
                self.log_enabled = False
                if self.log_file:
                    self.log_file.close()
                    self.log_file = None
                self.log_btn.setChecked(False)
                self.log_status_label.setText("Log: OFF")
                self.log_status_label.setStyleSheet("color: #444455; font-size: 10px;")
        
        def create_raw_data_panel(self):
            group = QGroupBox("SYSTEM MONITOR")
            layout = QHBoxLayout(group)
            layout.setSpacing(6)
            layout.setContentsMargins(3, 10, 3, 3)

            # dummy sparklines (tidak ditampilkan, cegah error update_display)
            self.spark_alt  = SparklineWidget("ALT",  "m",   "#00ff88")
            self.spark_vvel = SparklineWidget("VVEL", "m/s", "#00d4ff")

            # -- LEFT: System Event Log -------------------------------------
            evlog_widget = QWidget()
            evlog_lay = QVBoxLayout(evlog_widget)
            evlog_lay.setSpacing(3)
            evlog_lay.setContentsMargins(0, 0, 0, 0)

            evlog_hdr = QLabel("EVENT LOG")
            evlog_hdr.setStyleSheet(
                "color:#0d5a8a; font-family:'Consolas'; font-size:9px; font-weight:bold; letter-spacing:2px;"
            )
            evlog_lay.addWidget(evlog_hdr)

            self.event_log = QTextEdit()
            self.event_log.setReadOnly(True)
            self.event_log.setStyleSheet(
                "QTextEdit { background-color:#02050a; border:1px solid #0a1e2e; "
                "font-family:'Consolas'; font-size:10px; color:#aabbcc; }"
            )
            evlog_lay.addWidget(self.event_log)

            ev_ctrl = QHBoxLayout()
            ev_ctrl.setSpacing(4)

            self.mute_btn = QPushButton("MUTE OFF")
            self.mute_btn.setCheckable(True)
            self.mute_btn.setFixedWidth(80)
            self.mute_btn.clicked.connect(self._toggle_mute)
            self.mute_btn.setStyleSheet(
                "QPushButton { color:#556677; border-color:#223344; font-size:10px; }"
                "QPushButton:checked { color:#ffaa00; border-color:#886600; background:#1a1100; }"
            )
            ev_ctrl.addWidget(self.mute_btn)

            test_btn = QPushButton("TEST ALARMS")
            test_btn.setFixedWidth(100)
            test_btn.clicked.connect(self.open_test_dialog)
            test_btn.setStyleSheet("color:#00aadd; border-color:#004466; font-size:10px;")
            ev_ctrl.addWidget(test_btn)

            att_lbl = QLabel("MAX ATT:")
            att_lbl.setStyleSheet("color:#ff9900; font-family:'Consolas'; font-size:9px;")
            ev_ctrl.addWidget(att_lbl)

            self.att_limit_input = QLineEdit(str(self.max_attitude_angle))
            self.att_limit_input.setFixedWidth(40)
            self.att_limit_input.setStyleSheet(
                "QLineEdit { background:#111; color:#ff9900; border:1px solid #664400; "
                "font-family:'Consolas'; font-size:9px; padding:1px 3px; }"
            )
            self.att_limit_input.returnPressed.connect(self._apply_att_limit)
            ev_ctrl.addWidget(self.att_limit_input)

            att_deg = QLabel("deg")
            att_deg.setStyleSheet("color:#664400; font-family:'Consolas'; font-size:9px;")
            ev_ctrl.addWidget(att_deg)

            clr_ev = QPushButton("CLEAR")
            clr_ev.setFixedWidth(55)
            clr_ev.clicked.connect(self.event_log.clear)
            clr_ev.setStyleSheet("color:#445566; border-color:#223344; font-size:10px;")
            ev_ctrl.addWidget(clr_ev)
            ev_ctrl.addStretch()
            evlog_lay.addLayout(ev_ctrl)
            layout.addWidget(evlog_widget, 3)

            # -- RIGHT: Raw packet data -------------------------------------
            raw_widget = QWidget()
            raw_lay = QVBoxLayout(raw_widget)
            raw_lay.setSpacing(3)
            raw_lay.setContentsMargins(0, 0, 0, 0)

            raw_hdr = QLabel("RAW PACKETS")
            raw_hdr.setStyleSheet(
                "color:#0d5a8a; font-family:'Consolas'; font-size:9px; font-weight:bold; letter-spacing:2px;"
            )
            raw_lay.addWidget(raw_hdr)

            self.raw_text = QTextEdit()
            self.raw_text.setReadOnly(True)
            raw_lay.addWidget(self.raw_text)

            raw_ctrl = QHBoxLayout()
            raw_ctrl.setSpacing(4)

            self.show_raw_cb = QCheckBox("Show")
            self.show_raw_cb.setChecked(True)
            raw_ctrl.addWidget(self.show_raw_cb)

            clr_raw = QPushButton("CLEAR")
            clr_raw.setFixedWidth(55)
            clr_raw.clicked.connect(self.raw_text.clear)
            clr_raw.setStyleSheet("color:#445566; border-color:#223344; font-size:10px;")
            raw_ctrl.addWidget(clr_raw)

            self.log_btn = QPushButton("[R] CSV LOG")
            self.log_btn.setCheckable(True)
            self.log_btn.clicked.connect(self.toggle_logging)
            self.log_btn.setStyleSheet(
                "QPushButton { color:#556677; border-color:#223344; font-size:10px; }"
                "QPushButton:checked { color:#ff4444; border-color:#aa0000; background:#130000; }"
            )
            raw_ctrl.addWidget(self.log_btn)

            self.log_status_label = QLabel("OFF")
            self.log_status_label.setStyleSheet("color:#334455; font-size:9px;")
            raw_ctrl.addWidget(self.log_status_label)
            raw_ctrl.addStretch()

            self.mission_btn = QPushButton("> START MISSION")
            self.mission_btn.clicked.connect(self.toggle_mission)
            self.mission_btn.setStyleSheet(
                "QPushButton { color:#00ff88; border-color:#00aa55; background:#001a0a; font-size:10px; }"
                "QPushButton:hover { background:#002a10; }"
            )
            raw_ctrl.addWidget(self.mission_btn)
            raw_lay.addLayout(raw_ctrl)
            layout.addWidget(raw_widget, 2)

            return group

        def _toggle_mute(self):
            muted = self.mute_btn.isChecked()
            AlarmSystem.set_mute(muted)
            self.mute_btn.setText("MUTE ON" if muted else "MUTE OFF")

        def _apply_att_limit(self):
            try:
                val = float(self.att_limit_input.text())
                if 1 <= val <= 180:
                    self.max_attitude_angle = val
                    self._alm_att_warn = False
            except ValueError:
                pass
            self.att_limit_input.setText(str(int(self.max_attitude_angle)))
            
        def create_sensor_panel(self):
            group = QGroupBox("INSTRUMENTS -- IMU & GPS")
            layout = QVBoxLayout(group)
            layout.setSpacing(4)
            layout.setContentsMargins(3, 10, 3, 3)

            # -- Attitude + Compass ----------------------------------------
            inst_row = QHBoxLayout()
            inst_row.setSpacing(4)
            self.attitude_widget = AttitudeWidget()
            self.attitude_widget.setFixedHeight(110)
            self.compass_widget  = CompassWidget()
            self.compass_widget.setFixedHeight(110)
            inst_row.addWidget(self.attitude_widget)
            inst_row.addWidget(self.compass_widget)
            layout.addLayout(inst_row)

            # -- IMU numeric row (compact, below instruments) ---------------
            imu_frame = QFrame()
            imu_frame.setFixedHeight(22)
            imu_frame.setStyleSheet(
                "QFrame { background-color: #060f1c; border: 1px solid #0d3a5c; border-radius: 3px; }"
            )
            imu_h = QHBoxLayout(imu_frame)
            imu_h.setContentsMargins(10, 0, 10, 0)
            imu_h.setSpacing(0)

            def _imu_pair(key, color):
                lk = QLabel(key + ":")
                lk.setStyleSheet(
                    "color: #2e6a8a; font-family:'Consolas'; font-size:9px; font-weight:bold; padding-right:3px;"
                )
                lv = QLabel("+0.0")
                lv.setStyleSheet(
                    "color:%s; font-weight:bold; font-family:'Consolas'; font-size:11px; padding-right:16px;" % color
                )
                return lk, lv

            lk_r, self.roll_val  = _imu_pair("ROLL",  "#ff7777")
            lk_p, self.pitch_val = _imu_pair("PITCH", "#77ff88")
            lk_y, self.yaw_val   = _imu_pair("YAW",   "#7788ff")
            for w in (lk_r, self.roll_val, lk_p, self.pitch_val, lk_y, self.yaw_val):
                imu_h.addWidget(w)
            imu_h.addStretch()
            layout.addWidget(imu_frame)

            # proxy BigValueWidgets (not shown, satisfy update_display refs)
            self.bv_roll  = BigValueWidget("ROLL",  "deg", "#ff7777")
            self.bv_pitch = BigValueWidget("PITCH", "deg", "#77ff88")
            self.bv_yaw   = BigValueWidget("YAW",   "deg", "#7788ff")

            # -- GPS block -------------------------------------------------
            gps_frame = QFrame()
            gps_frame.setStyleSheet(
                "QFrame { background-color: #060f1c; border: 1px solid #0d3a5c; border-radius: 4px; }"
            )
            gf = QGridLayout(gps_frame)
            gf.setContentsMargins(8, 5, 8, 5)
            gf.setSpacing(4)

            def _lbl(t):
                l = QLabel(t)
                l.setStyleSheet(
                    "color: #2e6a8a; font-family: 'Consolas'; font-size: 9px; font-weight: bold;"
                )
                return l

            self.lat_label = QLabel("0.000000")
            self.lat_label.setStyleSheet("color: #00ff88; font-weight: bold; font-family: 'Consolas'; font-size: 10px;")
            self.lon_label = QLabel("0.000000")
            self.lon_label.setStyleSheet("color: #00ff88; font-weight: bold; font-family: 'Consolas'; font-size: 10px;")
            self.sats_label = QLabel("0")
            self.sats_label.setStyleSheet("color: #ffdd00; font-weight: bold; font-family: 'Consolas'; font-size: 10px;")
            self.gps_spd_label = QLabel("0.0 km/h")
            self.gps_spd_label.setStyleSheet("color: #00ccff; font-weight: bold; font-family: 'Consolas'; font-size: 10px;")

            gf.addWidget(_lbl("LAT"),  0, 0); gf.addWidget(self.lat_label,      0, 1)
            gf.addWidget(_lbl("LON"),  1, 0); gf.addWidget(self.lon_label,      1, 1)
            gf.addWidget(_lbl("SATS"), 0, 2); gf.addWidget(self.sats_label,     0, 3)
            gf.addWidget(_lbl("SPD"),  1, 2); gf.addWidget(self.gps_spd_label, 1, 3)
            layout.addWidget(gps_frame)

            self.lora_debug_label = QLabel("Last update: --")
            self.lora_debug_label.setStyleSheet(
                "color: #3a7a9a; font-family: 'Consolas'; font-size: 9px; padding: 2px;"
            )
            layout.addWidget(self.lora_debug_label)

            # proxy labels
            self.roll_label  = QLabel()
            self.pitch_label = QLabel()
            self.yaw_label   = QLabel()

            return group

        def create_flight_state_panel(self):
            _STATES = [
                "PRE-FLIGHT", "ARMED", "BOOST/ASCENT", "COAST",
                "APOGEE", "DROGUE DEPLOY", "MAIN DEPLOY", "TOUCHDOWN",
            ]
            frame = QFrame()
            frame.setFixedHeight(30)
            frame.setStyleSheet(
                "QFrame { background-color: #04080f; border: 1px solid #0d3a5c; border-radius: 4px; }"
            )
            lay = QHBoxLayout(frame)
            lay.setContentsMargins(10, 3, 10, 3)
            lay.setSpacing(2)

            phase_lbl = QLabel("FLIGHT PHASE :")
            phase_lbl.setStyleSheet(
                "color: #2e6a8a; font-family: 'Consolas'; font-size: 9px; "
                "font-weight: bold; background: transparent; border: none;"
            )
            lay.addWidget(phase_lbl)
            lay.addSpacing(6)

            self.state_labels = []
            for i, name in enumerate(_STATES):
                if i > 0:
                    arrow = QLabel("?")
                    arrow.setAlignment(Qt.AlignCenter)
                    arrow.setStyleSheet(
                        "color: #0d3a5c; font-size: 8px; background: transparent; border: none;"
                    )
                    lay.addWidget(arrow)
                lbl = QLabel(name)
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setFixedHeight(22)
                lbl.setStyleSheet(
                    "color: #1e3a4a; background-color: #020508; "
                    "border: 1px solid #0a1e2e; border-radius: 3px; "
                    "font-family: 'Consolas'; font-size: 9px; font-weight: bold; padding: 0 5px;"
                )
                self.state_labels.append(lbl)
                lay.addWidget(lbl)

            lay.addStretch()
            self._refresh_state_labels()
            return frame

        def _refresh_state_labels(self):
            for i, lbl in enumerate(self.state_labels):
                if i < self.flight_state:
                    lbl.setStyleSheet(
                        "color: #004d3a; background-color: #001a10; "
                        "border: 1px solid #003322; border-radius: 3px; "
                        "font-family: 'Consolas'; font-size: 9px; font-weight: bold; padding: 0 5px;"
                    )
                elif i == self.flight_state:
                    lbl.setStyleSheet(
                        "color: #001a22; background-color: #00ccff; "
                        "border: 1px solid #00ccff; border-radius: 3px; "
                        "font-family: 'Consolas'; font-size: 9px; font-weight: bold; padding: 0 5px;"
                    )
                else:
                    lbl.setStyleSheet(
                        "color: #1e3a4a; background-color: #020508; "
                        "border: 1px solid #0a1e2e; border-radius: 3px; "
                        "font-family: 'Consolas'; font-size: 9px; font-weight: bold; padding: 0 5px;"
                    )

        def _update_flight_state(self):
            data = self.serial.data
            vv   = self.vertical_velocity
            alt  = data.altitude
            prev = self.flight_state

            if self.flight_state == 0:    # PRE-FLIGHT ? ARMED
                if data.packet_count > 0 and data.lora_ok and data.sw1:
                    self.flight_state = 1
            elif self.flight_state == 1:  # ARMED ? BOOST/ASCENT
                if vv > 8.0:
                    self.flight_state = 2
            elif self.flight_state == 2:  # BOOST/ASCENT ? COAST
                if 0.0 < vv <= 8.0:
                    self.flight_state = 3
                elif vv <= 0.0 and alt > 5.0:
                    self.flight_state = 4
            elif self.flight_state == 3:  # COAST ? APOGEE
                if self.apogee_reached or vv <= 0.0:
                    self.flight_state = 4
            elif self.flight_state == 4:  # APOGEE ? DROGUE DEPLOY
                if vv < -3.0:
                    self.flight_state = 5
            elif self.flight_state == 5:  # DROGUE DEPLOY ? MAIN DEPLOY
                if 0.0 < alt < 100.0:
                    self.flight_state = 6
            elif self.flight_state == 6:  # MAIN DEPLOY ? TOUCHDOWN
                if alt < 5.0 and abs(vv) < 1.5:
                    self.flight_state = 7

            if self.flight_state != prev:
                self._refresh_state_labels()

        def create_video_panel(self):
            group = QGroupBox("VIDEO FEED")
            layout = QVBoxLayout(group)
            layout.setContentsMargins(3, 10, 3, 3)
            layout.setSpacing(4)

            self.video_widget = VideoStreamingWidget()
            layout.addWidget(self.video_widget)

            ctrl_frame = QFrame()
            ctrl_frame.setStyleSheet("QFrame { background-color: #04080f; border-radius: 4px; }")
            ctrl_layout = QHBoxLayout(ctrl_frame)
            ctrl_layout.setContentsMargins(8, 4, 8, 4)
            ctrl_layout.setSpacing(8)

            self.video_status = QLabel("Stream: Offline")
            self.video_status.setStyleSheet("color: #445566; font-family: 'Consolas'; font-size: 10px;")
            ctrl_layout.addWidget(self.video_status)

            self.btn_stream = QPushButton("> START STREAM")
            self.btn_stream.clicked.connect(self.toggle_stream)
            self.btn_stream.setStyleSheet(
                "QPushButton { color: #00ccff; border-color: #004466; }"
                "QPushButton:hover { border-color: #00ccff; }"
            )
            ctrl_layout.addWidget(self.btn_stream)

            self.btn_record = QPushButton("[R] RECORD")
            self.btn_record.setCheckable(True)
            self.btn_record.clicked.connect(self.toggle_record)
            self.btn_record.setEnabled(False)
            self.btn_record.setStyleSheet(
                "QPushButton { color: #555566; border-color: #222233; }"
                "QPushButton:enabled { color: #aaaaaa; border-color: #444455; }"
                "QPushButton:checked { color: #ff3333; border-color: #aa0000; "
                "  background-color: #1a0000; font-weight: bold; }"
                "QPushButton:checked:hover { background-color: #220000; }"
            )
            ctrl_layout.addWidget(self.btn_record)

            # Tombol LOCK TARGET → kirim perintah ke K230 via uplink
            self.btn_lock = QPushButton("[O] LOCK TARGET")
            self.btn_lock.setCheckable(True)
            self.btn_lock.clicked.connect(self.toggle_lock)
            self.btn_lock.setStyleSheet(
                "QPushButton { color: #ffaa00; border-color: #664400; }"
                "QPushButton:hover { border-color: #ffaa00; }"
                "QPushButton:checked { color: #ff3333; border-color: #aa0000; "
                "  background-color: #1a0000; font-weight: bold; }"
                "QPushButton:checked:hover { background-color: #220000; }"
            )
            ctrl_layout.addWidget(self.btn_lock)

            ctrl_layout.addStretch()

            self.rec_status_label = QLabel("")
            self.rec_status_label.setStyleSheet("color: #ff3333; font-family: 'Consolas'; font-size: 10px;")
            ctrl_layout.addWidget(self.rec_status_label)

            self.video_fps_label = QLabel("FPS: --")
            self.video_fps_label.setStyleSheet("color: #004466; font-family: 'Consolas'; font-size: 10px;")
            ctrl_layout.addWidget(self.video_fps_label)

            layout.addWidget(ctrl_frame)
            return group

        def toggle_stream(self):
            if not self.video_widget.is_streaming:
                self.video_widget.start_stream()
                self.btn_stream.setText("[] STOP STREAM")
                self.btn_stream.setStyleSheet(
                    "QPushButton { color: #ff4444; border-color: #aa2222; }"
                )
                self.video_status.setText("Stream: LIVE")
                self.btn_record.setEnabled(True)
            else:
                if self.video_widget.is_recording:
                    self.toggle_record()
                self.video_widget.stop_stream()
                self.btn_stream.setText("> START STREAM")
                self.btn_stream.setStyleSheet(
                    "QPushButton { color: #00ccff; border-color: #004466; }"
                    "QPushButton:hover { border-color: #00ccff; }"
                )
                self.video_status.setText("Stream: Offline")
                self.btn_record.setEnabled(False)
                self.btn_record.setChecked(False)
                self.rec_status_label.setText("")

        def toggle_record(self):
            if not self.video_widget.is_recording:
                if not OPENCV_AVAILABLE:
                    self.rec_status_label.setText("OpenCV not available")
                    return
                ok = self.video_widget.start_recording()
                if ok:
                    self.btn_record.setChecked(True)
                    self.rec_status_label.setText("REC: %s" % self.video_widget.record_filename)
                else:
                    self.btn_record.setChecked(False)
                    self.rec_status_label.setText("ERR: cannot open writer")
            else:
                self.video_widget.stop_recording()
                self.btn_record.setChecked(False)
                frames = self.video_widget.record_frame_count
                self.rec_status_label.setText("Saved %d frames" % frames)

        def toggle_lock(self):
            self.btn_lock.setText("[#] TARGET LOCKED" if self.btn_lock.isChecked()
                                  else "[O] LOCK TARGET")
            self.send_lock_state()   # kirim segera

        def send_lock_state(self):
            # Lock aktif kalau tombol ditekan ATAU switch SW5 ON.
            # Dikirim berulang (~5x/dtk) supaya tahan paket hilang di RF.
            sw5 = self.serial.data.sw5 if self.serial else False
            active = self.btn_lock.isChecked() or sw5
            try:
                self.lock_sock.sendto(b"LOCK 1" if active else b"LOCK 0", self.lock_addr)
            except Exception:
                pass

        def connect_signals(self):
            self.serial.signals.data_received.connect(self.on_data_received)
            self.serial.signals.connection_changed.connect(self.on_connection_changed)
            self.serial.signals.error_occurred.connect(self.on_error)
            
        def refresh_ports(self):
            self.port_combo.clear()
            ports = SerialHandler.list_ports()
            for port in ports:
                self.port_combo.addItem(port['device'], port['device'])
                
        def toggle_connection(self):
            if self.serial.is_connected:
                self.serial.disconnect()
            else:
                port = self.port_combo.currentData()
                if port:
                    self.serial.connect(port)
                    
        def on_connection_changed(self, connected):
            if connected:
                self.status_label.setText("[ CONNECTED ]")
                self.status_label.setStyleSheet("color: #00ccff; font-weight: bold; font-family: 'Consolas';")
                self.connect_btn.setText("Disconnect")
            else:
                self.status_label.setText("[ DISCONNECTED ]")
                self.status_label.setStyleSheet("color: #ff4444; font-weight: bold; font-family: 'Consolas';")
                self.connect_btn.setText("Connect")
                
        def on_data_received(self, line):
            if self.show_raw_cb.isChecked():
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self.raw_text.append("[%s] %s" % (timestamp, line))
                scrollbar = self.raw_text.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
                
        def on_error(self, error):
            self.status_label.setText("[ ERROR: %s ]" % error)
            self.status_label.setStyleSheet("color: #ff8800; font-weight: bold; font-family: 'Consolas';")
            
        def update_display(self):
            data = self.serial.data

            # Mission timer
            if self.mission_start_time is not None:
                elapsed = time.time() - self.mission_start_time
                h = int(elapsed // 3600)
                m = int((elapsed % 3600) // 60)
                s = int(elapsed % 60)
                self.mission_timer_label.setText("T+  %02d:%02d:%02d" % (h, m, s))

            # Vertical velocity (smoothed)
            now = time.time()
            dt = now - self.prev_alt_time
            if dt >= 0.2 and data.packet_count > 0:
                raw_vvel = (data.altitude - self.prev_altitude) / dt
                self.vertical_velocity = self.vertical_velocity * 0.6 + raw_vvel * 0.4
                self.prev_altitude = data.altitude
                self.prev_alt_time = now

            # Max altitude & apogee detection
            if data.altitude > self.max_altitude:
                self.max_altitude = data.altitude
                self.apogee_reached = False
            elif (self.max_altitude > 5.0 and
                  self.vertical_velocity < -1.0 and
                  not self.apogee_reached):
                self.apogee_reached = True

            self.maxalt_label.setText("MAX ALT: %.1f m" % self.max_altitude)

            vvel_txt = "%+.1f m/s" % self.vertical_velocity
            self.vvel_header_label.setText("V-VEL: %s" % vvel_txt)
            self.vvel_label.setText(vvel_txt)
            if self.vertical_velocity > 1.0:
                self.vvel_label.setStyleSheet("color: #00ffaa; font-weight: bold; font-family: 'Consolas';")
            elif self.vertical_velocity < -1.0:
                self.vvel_label.setStyleSheet("color: #ff6644; font-weight: bold; font-family: 'Consolas';")
            else:
                self.vvel_label.setStyleSheet("color: #888888; font-weight: bold; font-family: 'Consolas';")

            if self.apogee_reached:
                self.apogee_label.setText("v  APOGEE REACHED  v")
                self.apogee_label.setStyleSheet(
                    "color: #ff9900; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #1a0800; border-radius: 4px; padding: 4px;"
                )
            else:
                self.apogee_label.setText("APOGEE: --")
                self.apogee_label.setStyleSheet(
                    "color: #333344; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #08080f; border-radius: 4px; padding: 4px;"
                )

            # Battery %
            batt_rng = self.cfg_volt_max - self.cfg_volt_min
            batt_pct = int(max(0, min(100, (data.voltage - self.cfg_volt_min) / batt_rng * 100))) if batt_rng > 0 else 0
            self.batt_label.setText("%d%%" % batt_pct)
            if batt_pct > self.cfg_volt_warn:
                self.batt_label.setStyleSheet("color: #00ff88; font-weight: bold; font-family: 'Consolas';")
            elif batt_pct > self.cfg_volt_crit:
                self.batt_label.setStyleSheet("color: #ffdd00; font-weight: bold; font-family: 'Consolas';")
            else:
                self.batt_label.setStyleSheet("color: #ff3333; font-weight: bold; font-family: 'Consolas';")

            # Data logging
            if self.log_enabled and self.log_file and data.packet_count > 0:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self.log_file.write(
                    "%s,%.2f,%.2f,%.2f,%.3f,%.1f,%.1f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.1f\n" % (
                        ts, data.roll, data.pitch, data.yaw,
                        data.voltage, data.current, data.power,
                        data.pressure, data.altitude, data.temperature,
                        data.gps_lat, data.gps_lon, data.gps_speed,
                        data.gps_satellites, data.compass_heading
                    )
                )

            self.packet_label.setText("PKT: %d" % data.packet_count)

            # PC battery indicator
            if PSUTIL_AVAILABLE:
                batt = psutil.sensors_battery()
                if batt is not None:
                    pct = int(batt.percent)
                    charging = batt.power_plugged
                    icon = "~" if charging else ("v" if pct > 20 else "!")
                    self.pc_batt_label.setText("PC [%s%d%%]" % (icon, pct))
                    if charging:
                        style_color = "#00ccff"
                        bg = "#001a2a"
                        border = "#004466"
                    elif pct > 50:
                        style_color = "#00ff88"
                        bg = "#001a0a"
                        border = "#004422"
                    elif pct > 20:
                        style_color = "#ffdd00"
                        bg = "#1a1400"
                        border = "#665500"
                    else:
                        style_color = "#ff3333"
                        bg = "#1a0000"
                        border = "#660000"
                    self.pc_batt_label.setStyleSheet(
                        "color: %s; font-family: 'Consolas'; font-size: 10px; font-weight: bold;"
                        "background-color: %s; border: 1px solid %s; border-radius: 3px; padding: 0 6px;"
                        % (style_color, bg, border)
                    )
                else:
                    self.pc_batt_label.setText("PC [AC]")
                    self.pc_batt_label.setStyleSheet(
                        "color: #00ccff; font-family: 'Consolas'; font-size: 10px; font-weight: bold;"
                        "background-color: #001a2a; border: 1px solid #004466; border-radius: 3px; padding: 0 6px;"
                    )
            
            self.joy1_widget.set_values(data.j1x, data.j1y, data.j1btn)
            self.joy2_widget.set_values(data.j2x, data.j2y, data.j2btn)
            
            switches = [data.sw1, data.sw2, data.sw3, data.sw4, data.sw5]
            for i, (indicator, on) in enumerate(zip(self.sw_indicators, switches)):
                if on:
                    indicator.setStyleSheet("background-color: #00ccff; border: 1px solid #00ccff; border-radius: 5px; color: #000000; font-family: 'Consolas'; font-weight: bold;")
                else:
                    indicator.setStyleSheet("background-color: #1a1a1a; border: 1px solid #333333; border-radius: 5px; color: #444444; font-family: 'Consolas'; font-weight: bold;")

            if data.btn_merah:
                self.btn_merah_indicator.setStyleSheet(
                    "background-color: #cc0000; border: 2px solid #ff2222; border-radius: 5px; "
                    "color: #ffffff; font-family: 'Consolas'; font-weight: bold; font-size: 8px;"
                )
            else:
                self.btn_merah_indicator.setStyleSheet(
                    "background-color: #1a0000; border: 1px solid #440000; border-radius: 5px; "
                    "color: #552222; font-family: 'Consolas'; font-weight: bold; font-size: 8px;"
                )

            if data.btn_hijau:
                self.btn_hijau_indicator.setStyleSheet(
                    "background-color: #00aa33; border: 2px solid #00ff55; border-radius: 5px; "
                    "color: #ffffff; font-family: 'Consolas'; font-weight: bold; font-size: 8px;"
                )
            else:
                self.btn_hijau_indicator.setStyleSheet(
                    "background-color: #001a00; border: 1px solid #004400; border-radius: 5px; "
                    "color: #225522; font-family: 'Consolas'; font-weight: bold; font-size: 8px;"
                )
                    
            # NRF panel: pakai sliding window quality (lebih akurat dari all-time rate)
            self.nrf_rate_bar.setValue(data.nrf_quality)
            rssi = data.nrf_rssi
            if rssi >= -70:
                rssi_col = "#00ff88"
            elif rssi >= -90:
                rssi_col = "#ffdd00"
            else:
                rssi_col = "#ff4444"
            self.nrf_rssi_label.setText("%ddBm" % rssi)
            self.nrf_rssi_label.setStyleSheet(
                "color: %s; font-family: 'Consolas'; font-size: 9px; min-width: 58px;" % rssi_col
            )
            self.nrf_count_label.setText("TX:%d OK:%d" % (data.nrf_tx, data.nrf_ok))

            if data.nrf_conn:
                self.nrf_conn_indicator.setText("ACK OK")
                self.nrf_conn_indicator.setStyleSheet(
                    "background-color: #003300; border: 1px solid #00aa44; border-radius: 3px; "
                    "color: #00ff66; font-family: 'Consolas'; font-size: 9px; font-weight: bold;"
                )
            else:
                self.nrf_conn_indicator.setText("NO ACK")
                self.nrf_conn_indicator.setStyleSheet(
                    "background-color: #1a0000; border: 1px solid #440000; border-radius: 3px; "
                    "color: #441111; font-family: 'Consolas'; font-size: 9px; font-weight: bold;"
                )
            
            if data.lora_ok:
                age = time.time() - data.lora_last_update if data.lora_last_update > 0 else 0
                self.lora_status.setText("  [O]  LoRa: LINK OK  --  %.1fs ago" % age)
                self.lora_status.setStyleSheet(
                    "color: #00ff88; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #001a0a; border: 1px solid #00aa44; border-radius: 3px; padding: 2px;"
                )
                self.lora_debug_label.setText("PKT OK | %.1fs ago" % age)
            elif data.nrf_conn and data.packet_count > 0:
                self.lora_status.setText("  [~]  LoRa: NO LINK  --  NRF RELAY")
                self.lora_status.setStyleSheet(
                    "color: #ffcc00; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #1a1200; border: 1px solid #886600; border-radius: 3px; padding: 2px;"
                )
                self.lora_debug_label.setText("Data via NRF relay")
            else:
                self.lora_status.setText("  [X]  LoRa: NO LINK  --  WAITING...")
                self.lora_status.setStyleSheet(
                    "color: #ff3333; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #130000; border: 1px solid #440000; border-radius: 3px; padding: 2px;"
                )
                self.lora_debug_label.setText("No data received")
                
            self.attitude_widget.set_attitude(data.roll, data.pitch)
            self.compass_widget.set_heading(
                data.compass_heading,
                data.compass_valid,
                has_heading=data.has_heading_data
            )

            # -- BigValueWidget updates --
            self.bv_altitude.set_value("%.1f" % data.altitude)
            self.bv_maxalt.set_value("%.1f" % self.max_altitude)

            vv = self.vertical_velocity
            vv_color = "#00ff88" if vv > 0.5 else "#ff6644" if vv < -0.5 else "#556677"
            self.bv_vvel.set_value("%+.1f" % vv, vv_color)

            volt_col = ("#00ff88" if batt_pct > self.cfg_volt_warn
                        else "#ffdd00" if batt_pct > self.cfg_volt_crit else "#ff3333")
            self.bv_voltage.set_value("%.2f" % data.voltage, volt_col)
            self.bv_current.set_value("%.1f" % (data.current / 1000.0))

            pwr_w = data.power / 1000.0
            pwr_col = ("#ff3333" if self.cfg_pwr_crit > 0 and pwr_w >= self.cfg_pwr_crit
                       else "#ffaa00" if self.cfg_pwr_warn > 0 and pwr_w >= self.cfg_pwr_warn
                       else "#ff44ff")
            self.bv_power.set_value("%.1f" % pwr_w, pwr_col)

            temp_col = ("#ff3333" if data.temperature >= self.cfg_temp_crit
                        else "#ff6600" if data.temperature >= self.cfg_temp_warn else "#ff9900")
            self.bv_temp.set_value("%.1f" % data.temperature, temp_col)

            pres_col = ("#ff4444" if self.cfg_pres_min > 0 and data.pressure < self.cfg_pres_min
                        else "#ff4444" if self.cfg_pres_max > 0 and data.pressure > self.cfg_pres_max
                        else "#88ff88")
            self.bv_pressure.set_value("%.0f" % data.pressure, pres_col)

            batt_col = "#00ff88" if batt_pct > self.cfg_volt_warn else "#ffdd00" if batt_pct > self.cfg_volt_crit else "#ff3333"
            self.bv_batt.set_value("%d" % batt_pct, batt_col)

            self.roll_val.setText("%+.1f" % data.roll)
            self.pitch_val.setText("%+.1f" % data.pitch)
            self.yaw_val.setText("%+.1f" % data.yaw)

            # -- Sparklines --
            if data.packet_count > 0:
                self.spark_alt.push(data.altitude)
                self.spark_vvel.push(self.vertical_velocity)

            # -- GPS labels --
            self.lat_label.setText("%.6f" % data.gps_lat)
            self.lon_label.setText("%.6f" % data.gps_lon)
            self.sats_label.setText("%d" % data.gps_satellites)
            self.gps_spd_label.setText("%.1f km/h" % data.gps_speed)

            if data.gps_valid:
                gps_c = "color: #00ff88; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                self.lat_label.setStyleSheet(gps_c)
                self.lon_label.setStyleSheet(gps_c)
                self.sats_label.setStyleSheet("color: #00ff00; font-weight: bold; font-family: 'Consolas'; font-size: 10px;")
                self.gps_label.setText("GPS: %.5f, %.5f (%d sats)" % (data.gps_lat, data.gps_lon, data.gps_satellites))
                self.gps_label.setStyleSheet("color: #00ff88; font-family: 'Consolas'; font-size: 10px;")
            else:
                bad = "color: #ff5555; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                self.lat_label.setStyleSheet(bad)
                self.lon_label.setStyleSheet(bad)
                self.sats_label.setStyleSheet(bad)
                self.gps_label.setText("GPS: NO FIX (%d sats)" % data.gps_satellites)
                self.gps_label.setStyleSheet("color: #ff5555; font-family: 'Consolas'; font-size: 10px;")
            
            self.map_widget.set_drone_position(
                lat=data.gps_lat,
                lon=data.gps_lon,
                heading=data.compass_heading,
                altitude=data.altitude,
                speed=data.gps_speed,
                gps_valid=data.gps_valid,
                satellites=data.gps_satellites
            )
            
            self.btn_auto.setChecked(self.map_widget.auto_center)

            # -- ALARM STATUS BAR ------------------------------------------
            active = AlarmSystem._loop_active
            if active:
                names = [AlarmSystem._LABELS.get(a, a.upper()) for a in active]
                alarm_txt = names[0] if len(names) == 1 else "%d ALARMS" % len(names)
                self.alarm_bar.setText("  [!] %s  " % alarm_txt)
                self.alarm_bar.setToolTip(" | ".join(names))
                self.alarm_bar.setStyleSheet(
                    "color: #ff2222; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #1a0000; border: 1px solid #cc0000; border-radius: 3px; padding: 0 6px;"
                )
            else:
                self.alarm_bar.setText("  NO ALARMS  ")
                self.alarm_bar.setStyleSheet(
                    "color: #005533; font-weight: bold; font-family: 'Consolas'; font-size: 10px;"
                    "background-color: #001a0a; border: 1px solid #003322; border-radius: 3px; padding: 0 6px;"
                )

            # -- ALARM STATE MACHINE (transition-based) --------------------
            if data.packet_count > 0:
                # LoRa link
                if not data.lora_ok and self._alm_lora_ok:
                    self.log_event('CRIT', 'LoRa signal lost', 'lora_lost')
                    self._alm_lora_ok = False
                elif data.lora_ok and not self._alm_lora_ok:
                    self.log_event('INFO', 'LoRa link restored', 'lora_ok')
                    self._alm_lora_ok = True

                # GPS fix
                if data.gps_valid and not self._alm_gps_valid:
                    self.log_event('INFO', 'GPS fix acquired -- %d satellites' % data.gps_satellites, 'gps_ok')
                    self._alm_gps_valid = True
                elif not data.gps_valid and self._alm_gps_valid:
                    self.log_event('WARN', 'GPS signal lost', 'gps_lost')
                    self._alm_gps_valid = False

                # Battery
                _brng = self.cfg_volt_max - self.cfg_volt_min
                batt_pct_chk = int(max(0, min(100, (data.voltage - self.cfg_volt_min) / _brng * 100))) if _brng > 0 else 0
                if batt_pct_chk <= self.cfg_volt_crit:
                    if not self._alm_batt_crit:
                        self.log_event('CRIT', 'Battery CRITICAL -- %.2fV (%d%%)' % (data.voltage, batt_pct_chk))
                        self._alm_batt_crit = True
                    AlarmSystem.loop_start('batt_crit')
                elif batt_pct_chk > self.cfg_volt_crit + 5:
                    self._alm_batt_crit = False
                    AlarmSystem.loop_stop('batt_crit')
                if batt_pct_chk <= self.cfg_volt_warn and not self._alm_batt_warn and not self._alm_batt_crit:
                    self.log_event('WARN', 'Battery low -- %.2fV (%d%%)' % (data.voltage, batt_pct_chk), 'batt_warn')
                    self._alm_batt_warn = True
                elif batt_pct_chk > self.cfg_volt_warn + 5:
                    self._alm_batt_warn = False

                # Temperature
                if data.temperature >= self.cfg_temp_crit:
                    if not self._alm_temp_crit:
                        self.log_event('CRIT', 'Temperature CRITICAL -- %.1f C' % data.temperature)
                        self._alm_temp_crit = True
                    AlarmSystem.loop_start('temp_crit')
                elif data.temperature < self.cfg_temp_crit - 5:
                    self._alm_temp_crit = False
                    AlarmSystem.loop_stop('temp_crit')
                if data.temperature >= self.cfg_temp_warn and not self._alm_temp_warn and not self._alm_temp_crit:
                    self.log_event('WARN', 'High temperature -- %.1f C' % data.temperature, 'temp_warn')
                    self._alm_temp_warn = True
                elif data.temperature < self.cfg_temp_warn - 5:
                    self._alm_temp_warn = False

                # Pressure (hanya jika cfg_pres_min/max > 0)
                pres_bad = ((self.cfg_pres_min > 0 and data.pressure < self.cfg_pres_min) or
                            (self.cfg_pres_max > 0 and data.pressure > self.cfg_pres_max))
                if pres_bad:
                    if not self._alm_pres_crit:
                        self.log_event('CRIT', 'Pressure out of range -- %.1f hPa' % data.pressure)
                        self._alm_pres_crit = True
                    AlarmSystem.loop_start('crit')
                else:
                    if self._alm_pres_crit:
                        AlarmSystem.loop_stop('crit')
                    self._alm_pres_crit = False

                # Power
                pwr_w = data.power / 1000.0
                if self.cfg_pwr_crit > 0 and pwr_w >= self.cfg_pwr_crit:
                    if not self._alm_pwr_crit:
                        self.log_event('CRIT', 'Power CRITICAL -- %.1f W' % pwr_w)
                        self._alm_pwr_crit = True
                    AlarmSystem.loop_start('crit')
                elif pwr_w < self.cfg_pwr_crit * 0.9 if self.cfg_pwr_crit > 0 else True:
                    if self._alm_pwr_crit:
                        AlarmSystem.loop_stop('crit')
                    self._alm_pwr_crit = False
                if self.cfg_pwr_warn > 0 and pwr_w >= self.cfg_pwr_warn and not self._alm_pwr_warn and not self._alm_pwr_crit:
                    self.log_event('WARN', 'High power -- %.1f W' % pwr_w, 'warn')
                    self._alm_pwr_warn = True
                elif self.cfg_pwr_warn > 0 and pwr_w < self.cfg_pwr_warn * 0.9:
                    self._alm_pwr_warn = False

                # High attitude (roll or pitch) -- loop tanpa jeda selama kondisi aktif
                att_bad = abs(data.roll) > self.max_attitude_angle or abs(data.pitch) > self.max_attitude_angle
                if att_bad:
                    if not self._alm_att_warn:
                        self.log_event('WARN', 'High attitude -- R:%+.1f P:%+.1f deg' % (data.roll, data.pitch))
                        self._alm_att_warn = True
                    AlarmSystem.loop_start('high_attitude')
                else:
                    self._alm_att_warn = False
                    AlarmSystem.loop_stop('high_attitude')

                # Apogee
                if self.apogee_reached and not hasattr(self, '_alm_apogee_fired'):
                    self.log_event('INFO', 'APOGEE detected -- max alt %.1fm' % self.max_altitude, 'apogee')
                    self._alm_apogee_fired = True
                if not self.apogee_reached:
                    self._alm_apogee_fired = False

            # Update video FPS label
            fps = self.video_widget.video_fps
            if fps > 0:
                self.video_fps_label.setText("FPS: %d" % fps)
                self.video_fps_label.setStyleSheet(
                    "color: #00ff88; font-family: 'Consolas'; font-size: 10px;"
                    if fps >= 20 else
                    "color: #ffcc00; font-family: 'Consolas'; font-size: 10px;"
                )
            else:
                self.video_fps_label.setText("FPS: --")
                self.video_fps_label.setStyleSheet("color: #334455; font-family: 'Consolas'; font-size: 10px;")

            # Update rec counter while recording
            if self.video_widget.is_recording:
                self.rec_status_label.setText(
                    "REC %d frm | %s" % (
                        self.video_widget.record_frame_count,
                        self.video_widget.record_filename
                    )
                )

            self._update_flight_state()

        def closeEvent(self, event):
            self.serial.disconnect()
            if self.video_widget.is_streaming:
                self.video_widget.stop_stream()
            # Terminate libmpv bersih agar tidak ada proses/thread tersisa
            try:
                if getattr(self.video_widget, 'mpv', None) is not None:
                    self.video_widget.mpv.terminate()
                    self.video_widget.mpv = None
            except Exception:
                pass
            if self.log_enabled and self.log_file:
                self.log_file.close()
            event.accept()


def main():
    if GUI_AVAILABLE and (len(sys.argv) < 2 or sys.argv[1] != '--terminal'):
        app = QApplication(sys.argv)
        # libmpv WAJIB LC_NUMERIC="C". QApplication mereset locale ke locale
        # sistem, jadi set ulang ke "C" di sini (pengaman tambahan).
        try:
            import locale
            locale.setlocale(locale.LC_NUMERIC, 'C')
        except Exception:
            pass
        login_window = LoginDialog()
        if login_window.exec_() == QDialog.Accepted:
            window = SpacebotGCS()
            window.showFullScreen()
            sys.exit(app.exec_())
        else:
            sys.exit(0)

if __name__ == "__main__":
    main()