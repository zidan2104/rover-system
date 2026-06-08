#!/usr/bin/env python3
import cv2, numpy as np, nncaseruntime as nn
import time, sys, threading, subprocess, queue, os, socket, json

# -- CONFIG ---------------------------------------------
KMODEL_PATH    = "/root/yolo11n.kmodel"
INPUT_SIZE     = 224  # ?? INI OBATNYA! Ternyata otaknya ukuran 224x224!
CONF_THRESH    = 0.35 # Karena 224x224 agak kecil, kita turunin pedenya dikit
IOU_THRESH     = 0.45
CAM_W, CAM_H   = 640, 480
STREAM_FPS     = 25
STREAM_PORT    = 8554
TELEMETRY_PORT = 5555

# -- TRACKING / LOCK ------------------------------------
CONTROL_PORT     = 5601    # UDP: terima perintah lock dari GCS (via uplink WFB / RNDIS)
AUTO_LOCK_DEMO   = False   # False = lock HANYA dari perintah GCS (tombol/SW5 via uplink)
                          # True  = auto-lock objek tengah (mode demo/uji tanpa uplink)
LOCK_LOST_FRAMES = 6       # frame tanpa match → TARGET LOST (recovery cepat)
LOCK_MAX_JUMP    = 160     # px: batas lompatan target antar frame (cegah loncat ke objek lain)
LOCK_EMA         = 0.5     # smoothing posisi 0..1 (kecil=halus, besar=responsif)

# -- KAMUS OBJEK YOLO (80 Kelas COCO) -------------------
CLASSES = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
           "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
           "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
           "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
           "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
           "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
           "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
           "hair drier", "toothbrush"]

latest_frame = [None]
frame_seq    = [0]      # counter frame kamera → loop proses tiap frame BARU (anti judder)
frame_lock   = threading.Lock()
vpu_queue    = queue.Queue(maxsize=2)   # penyangga 1 frame → serap jitter inference (anti patah-patah)
latest_telemetry = {"fps":0,"inf_ms":0,"det":0,"objects":[]}

# Perintah lock dari luar (GCS). Diset oleh control_thread / auto-demo.
lock_ctl = {"requested": False}

# -- PERFORMANCE MODE -----------------------------------
def set_performance_mode():
    gov_paths = [
        "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor",
        "/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor",
        "/sys/devices/system/cpu/cpufreq/policy0/scaling_governor",
        "/sys/devices/system/cpu/cpufreq/policy1/scaling_governor",
    ]
    for path in gov_paths:
        if os.path.exists(path):
            try:
                with open(path, "w") as f: f.write("performance\n")
            except: pass

# -- TELEMETRY THREAD -----------------------------------
def telemetry_thread():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', TELEMETRY_PORT)); srv.listen(1)
    print(f"[TELEM] Port {TELEMETRY_PORT} siap ngirim data ke GCS")
    while True:
        cli, addr = srv.accept()
        cli.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:
                cli.sendall((json.dumps(latest_telemetry)+"\n").encode())
                time.sleep(0.05)
        except: pass
        finally: cli.close()

# -- CONTROL THREAD (terima perintah lock dari GCS) -----
def control_thread():
    # GCS kirim STATE berulang via UDP: "LOCK 1" / "LOCK 0".
    # Pakai state (bukan edge) → tahan paket hilang di link RF.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', CONTROL_PORT))
    print(f"[CTRL] UDP {CONTROL_PORT} siap terima perintah lock")
    while True:
        try:
            data, _ = s.recvfrom(64)
            txt = data.decode(errors='ignore').strip().upper()
            if txt.startswith("LOCK"):
                lock_ctl["requested"] = txt.endswith("1")
        except: pass

# -- CAMERA THREAD -------------------------------------
def capture_thread():
    cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    if not cap.isOpened():
        print("[FATAL] Kamera gagal"); sys.exit(1)
    print("[CAM] OK")
    while True:
        ret, frame = cap.read()
        if not ret: continue
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with frame_lock:
            latest_frame[0] = frame
            frame_seq[0] += 1

# -- VPU THREAD (NON BLOCKING STREAM) ------------------
def start_ffmpeg():
    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", f"{CAM_W}x{CAM_H}",
        "-r", str(STREAM_FPS),
        "-i", "-",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-pix_fmt", "yuv420p",
        "-c:v", "h264_v4l2m2m",
        "-b:v", "300k",
        "-maxrate", "300k",       # cap spike bitrate saat gerak cepat (anti patah)
        "-bufsize", "300k",       # VBV ketat = CBR konsisten, latency tetap rendah
        "-g", "10",               # keyframe lebih jarang → tak ada spike I-frame tiap 5
        "-bf", "0",
        "-sc_threshold", "0",     # matikan scene-cut → tak nyelip keyframe saat gerak
        "-f", "h264",
        # tcp_nodelay=1: matikan Nagle → paket H264 langsung dikirim (low latency)
        f"tcp://0.0.0.0:{STREAM_PORT}?listen=1&tcp_nodelay=1"
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

def vpu_thread():
    pipe = start_ffmpeg()
    print(f"[VPU] FFmpeg TCP port {STREAM_PORT}")
    while True:
        frame = vpu_queue.get()
        try:
            pipe.stdin.write(frame.tobytes())
        except:
            pipe = start_ffmpeg()

# -- NMS -----------------------------------------------
def nms_numpy(boxes, scores, iou_thresh):
    if len(boxes) == 0: return []
    x1 = boxes[:,0] - boxes[:,2]/2; y1 = boxes[:,1] - boxes[:,3]/2
    x2 = boxes[:,0] + boxes[:,2]/2; y2 = boxes[:,1] + boxes[:,3]/2
    areas = (x2-x1)*(y2-y1); order = scores.argsort()[::-1]; keep = []
    while order.size > 0:
        i = order[0]; keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]]); yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]]); yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2-xx1); h = np.maximum(0.0, yy2-yy1)
        iou = (w*h) / (areas[i] + areas[order[1:]] - w*h + 1e-6)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return keep

# -- MAIN ----------------------------------------------
print("[START] Spacebot K230 AI FPV (YOLO11 224x224 Edition)")
set_performance_mode()

threading.Thread(target=capture_thread, daemon=True).start()
threading.Thread(target=telemetry_thread, daemon=True).start()
threading.Thread(target=vpu_thread, daemon=True).start()
threading.Thread(target=control_thread, daemon=True).start()

time.sleep(2)

print(f"[KPU] Load model {KMODEL_PATH}...")
with open(KMODEL_PATH, "rb") as f: kdata = f.read()
interp = nn.Interpreter(); interp.load_model(kdata)
print("[KPU] OK")

fc = 0; fps = 0; t0 = time.time()
last_seq = -1     # frame kamera terakhir yang sudah diproses

# State lock: tracking-by-detection pakai KPU, posisi di-smooth EMA
track = {"active": False, "cls": None,
         "cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0, "miss": 0}

while True:
    with frame_lock:
        frame = latest_frame[0]
        seq   = frame_seq[0]
    # Pace = laju kamera: hanya proses kalau ada frame BARU (mulus, anti judder)
    if frame is None or seq == last_seq:
        time.sleep(0.002)
        continue
    last_seq = seq

    requested = True if AUTO_LOCK_DEMO else lock_ctl["requested"]
    cxc, cyc = CAM_W / 2.0, CAM_H / 2.0
    icx, icy = int(cxc), int(cyc)

    display = frame.copy()
    objs = []

    # ===================== YOLO (KPU) TIAP FRAME =====================
    t_inf = time.time()
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    inp = np.ascontiguousarray(np.transpose(img, (2,0,1))[np.newaxis].astype(np.uint8))
    tensor = nn.RuntimeTensor.from_numpy(inp)
    interp.set_input_tensor(0, tensor); interp.run()
    raw = interp.get_output_tensor(0).to_numpy()[0]
    data2 = raw.transpose()
    boxes = data2[:, :4]
    scores = np.max(data2[:, 4:], axis=1)
    class_ids = np.argmax(data2[:, 4:], axis=1)
    mask = scores > CONF_THRESH
    vb, vs, vcls = boxes[mask], scores[mask], class_ids[mask]
    keep = nms_numpy(vb, vs, IOU_THRESH)
    inf_ms = (time.time() - t_inf) * 1000
    ndet = len(keep)

    # Susun deteksi (ruang kamera 640x480)
    dets = []
    for i in keep:
        bx, by, bw, bh = vb[i]
        x1 = int((bx - bw/2) * CAM_W / INPUT_SIZE); y1 = int((by - bh/2) * CAM_H / INPUT_SIZE)
        x2 = int((bx + bw/2) * CAM_W / INPUT_SIZE); y2 = int((by + bh/2) * CAM_H / INPUT_SIZE)
        x1 = max(0, x1); y1 = max(0, y1); x2 = min(CAM_W-1, x2); y2 = min(CAM_H-1, y2)
        dets.append({"x1":x1, "y1":y1, "x2":x2, "y2":y2,
                     "ccx":(x1+x2)/2.0, "ccy":(y1+y2)/2.0,
                     "w":x2-x1, "h":y2-y1, "cls":int(vcls[i]), "s":float(vs[i])})

    # ===================== LOCK: acquire / follow (dari hasil KPU) ====
    locked = None
    if not requested:
        track["active"] = False
    elif not track["active"]:
        # ACQUIRE: deteksi terdekat ke crosshair tengah
        if dets:
            d0 = min(dets, key=lambda d: (d["ccx"]-cxc)**2 + (d["ccy"]-cyc)**2)
            track.update(active=True, cls=d0["cls"],
                         cx=d0["ccx"], cy=d0["ccy"], w=d0["w"], h=d0["h"], miss=0)
            locked = d0
    else:
        # FOLLOW: kelas sama, terdekat ke posisi (smoothed) terakhir, dalam batas lompatan
        cand = [d for d in dets if d["cls"] == track["cls"]]
        best = None
        if cand:
            best = min(cand, key=lambda d: (d["ccx"]-track["cx"])**2 + (d["ccy"]-track["cy"])**2)
            if ((best["ccx"]-track["cx"])**2 + (best["ccy"]-track["cy"])**2) ** 0.5 > LOCK_MAX_JUMP:
                best = None
        if best is not None:
            a = LOCK_EMA                       # smoothing posisi -> reticle halus
            track["cx"] = a*best["ccx"] + (1-a)*track["cx"]
            track["cy"] = a*best["ccy"] + (1-a)*track["cy"]
            track["w"]  = a*best["w"]   + (1-a)*track["w"]
            track["h"]  = a*best["h"]   + (1-a)*track["h"]
            track["miss"] = 0
            locked = best
        else:
            track["miss"] += 1
            if track["miss"] > LOCK_LOST_FRAMES:
                track["active"] = False        # TARGET LOST -> re-acquire frame berikut

    # ===================== GAMBAR =====================
    for d in dets:
        is_lock = (d is locked)
        col = (0, 0, 255) if is_lock else (0, 255, 0)
        th  = 2 if is_lock else 1
        nama = CLASSES[d["cls"]] if d["cls"] < len(CLASSES) else f"Obj-{d['cls']}"
        cv2.rectangle(display, (d["x1"], d["y1"]), (d["x2"], d["y2"]), col, th)
        label = (f"LOCKED {nama} {d['s']:.2f}") if is_lock else (f"{nama} {d['s']:.2f}")
        cv2.putText(display, label, (d["x1"], max(d["y1"]-5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, th)
        objs.append({"x":d["x1"], "y":d["y1"], "w":d["w"], "h":d["h"],
                     "s":round(d["s"],2), "nama":nama})

    # Crosshair tengah (acuan lock)
    cv2.line(display, (icx-15, icy), (icx+15, icy), (255, 255, 0), 1)
    cv2.line(display, (icx, icy-15), (icx, icy+15), (255, 255, 0), 1)

    # Reticle target (di posisi SMOOTHED) + info lock
    lock_info = {"active": False, "lost": False}
    if track["active"]:
        tx, ty = int(track["cx"]), int(track["cy"])
        dx, dy = int(track["cx"] - cxc), int(track["cy"] - cyc)
        lost = track["miss"] > 0
        cv2.circle(display, (tx, ty), 16, (0, 0, 255), 2)
        cv2.line(display, (icx, icy), (tx, ty), (0, 0, 255), 1)
        nm = CLASSES[track["cls"]] if (track["cls"] is not None and track["cls"] < len(CLASSES)) else "target"
        status = "LOST" if lost else "LOCKED"
        cv2.putText(display, f"{status} {nm} dx:{dx} dy:{dy}", (5, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        lock_info = {"active": True, "lost": lost, "cls": nm,
                     "x": tx, "y": ty, "dx": dx, "dy": dy}

    mode_txt = "LOCK" if track["active"] else "SCAN"
    cv2.putText(display, f"{mode_txt} | {fps:.1f} FPS | KPU: {inf_ms:.0f}ms", (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    latest_telemetry = {"fps":round(fps,1), "inf_ms":round(inf_ms,1), "det":ndet,
                        "objects":objs, "lock":lock_info, "mode":mode_txt}

    # -- PUSH KE VPU --
    # 1 frame per frame-kamera (loop sudah di-pace ke laju kamera di awal),
    # langsung tanpa nahan = latency rendah, dan tidak spin/overfeed di TRACK.
    if not vpu_queue.full():
        vpu_queue.put(display)
    fc += 1
    if time.time() - t0 >= 1:
        fps = fc / (time.time() - t0)
        fc = 0; t0 = time.time()
