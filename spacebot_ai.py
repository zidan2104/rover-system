#!/usr/bin/env python3
import cv2, numpy as np, nncaseruntime as nn
import time, sys, threading, subprocess, queue, os, socket, json
from nanotrack_kpu import NanoTrackKPU   # tracker full-KPU (NanoTrack)

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
LOCK_LOST_FRAMES = 8       # frame skor rendah berturut → TARGET LOST → balik DETECT

# -- NANOTRACK (tracker FULL-KPU) -----------------------
NANO_CROP_KMODEL = "/root/cropped_test127.kmodel"            # template @127
NANO_SRC_KMODEL  = "/root/nanotrack_backbone_sim.kmodel"     # search   @255
NANO_HEAD_KMODEL = "/root/nanotracker_head_calib_k230.kmodel" # head
NANO_THRESH      = 0.10    # skor min masih nge-lock (referensi C++ pakai 0.1)

# -- TAKTIS (auto re-lock + prediksi alpha-beta + HUD) --
ALPHA            = 0.5     # alpha-beta filter: bobot koreksi POSISI (haluskan)
BETA             = 0.12    # alpha-beta filter: bobot koreksi KECEPATAN (prediksi)
LEAD_SCALE       = 8.0     # panjang garis lead = kecepatan × skala
RELOCK_AFTER     = 4       # frame hilang sebelum mulai cari ulang (YOLO)
RELOCK_EVERY     = 3       # saat hilang, jalankan YOLO re-lock tiap N frame
RELOCK_RADIUS    = 130     # px: re-lock hanya objek SEKELAS dalam radius prediksi

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

def iou_box(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    """IoU dua kotak → seberapa tumpang-tindih (1=sama persis, 0=tidak overlap)."""
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
    return inter / ua if ua > 0 else 0.0

# -- YOLO detect (dipakai SCAN & auto re-lock) ---------
def yolo_detect(frame):
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    inp = np.ascontiguousarray(np.transpose(img, (2,0,1))[np.newaxis].astype(np.uint8))
    interp.set_input_tensor(0, nn.RuntimeTensor.from_numpy(inp)); interp.run()
    raw = interp.get_output_tensor(0).to_numpy()[0]
    data2 = raw.transpose()
    boxes = data2[:, :4]
    scores = np.max(data2[:, 4:], axis=1)
    class_ids = np.argmax(data2[:, 4:], axis=1)
    mask = scores > CONF_THRESH
    vb, vs, vcls = boxes[mask], scores[mask], class_ids[mask]
    keep = nms_numpy(vb, vs, IOU_THRESH)
    out = []
    for i in keep:
        bx0, by0, bw0, bh0 = vb[i]
        x1 = int((bx0-bw0/2)*CAM_W/INPUT_SIZE); y1 = int((by0-bh0/2)*CAM_H/INPUT_SIZE)
        x2 = int((bx0+bw0/2)*CAM_W/INPUT_SIZE); y2 = int((by0+bh0/2)*CAM_H/INPUT_SIZE)
        x1 = max(0,x1); y1 = max(0,y1); x2 = min(CAM_W-1,x2); y2 = min(CAM_H-1,y2)
        ci = int(vcls[i])
        out.append({"x1":x1, "y1":y1, "w":x2-x1, "h":y2-y1,
                    "ccx":(x1+x2)/2.0, "ccy":(y1+y2)/2.0, "cls":ci, "s":float(vs[i]),
                    "nama":CLASSES[ci] if ci < len(CLASSES) else "obj-%d" % ci})
    return out

# -- HUD taktis ----------------------------------------
def draw_tactical(display, box, vx, vy, score, status, cxc, cyc, icx, icy):
    x, y, w, h = int(box[0]), int(box[1]), int(box[2]), int(box[3])
    x2 = x + w; y2 = y + h
    tx = x + w//2; ty = y + h//2
    if status == "LOCKED":    col = (0, 0, 255)
    elif status == "RE-LOCK": col = (0, 255, 255)
    else:                     col = (0, 140, 255)
    # corner bracket (reticle militer)
    L = max(8, int(min(w, h) * 0.28))
    for (cx_, cy_, sx, sy) in ((x,y,1,1),(x2,y,-1,1),(x,y2,1,-1),(x2,y2,-1,-1)):
        cv2.line(display, (cx_, cy_), (cx_+sx*L, cy_), col, 2)
        cv2.line(display, (cx_, cy_), (cx_, cy_+sy*L), col, 2)
    # center reticle
    cv2.line(display, (tx-10, ty), (tx+10, ty), col, 1)
    cv2.line(display, (tx, ty-10), (tx, ty+10), col, 1)
    cv2.circle(display, (tx, ty), 3, col, -1)
    # garis crosshair -> target
    cv2.line(display, (icx, icy), (tx, ty), col, 1)
    # lead arrow (arah gerak)
    lx = int(tx + vx*LEAD_SCALE); ly = int(ty + vy*LEAD_SCALE)
    if abs(lx-tx) > 3 or abs(ly-ty) > 3:
        cv2.arrowedLine(display, (tx, ty), (lx, ly), (0, 255, 0), 1, tipLength=0.3)
    # bar confidence
    bx0, by0, bw0 = 5, 46, 90
    cv2.rectangle(display, (bx0, by0), (bx0+bw0, by0+6), (50, 50, 50), -1)
    cv2.rectangle(display, (bx0, by0), (bx0+int(bw0*min(1.0, score/0.5)), by0+6), col, -1)
    dx = tx - int(cxc); dy = ty - int(cyc)
    cv2.putText(display, "%s conf:%.2f dx:%d dy:%d" % (status, score, dx, dy),
                (5, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

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

# Tracker FULL-KPU (NanoTrack: crop@127 + src@255 + head)
print("[NANO] Load tracker kmodels...")
tracker = NanoTrackKPU(NANO_CROP_KMODEL, NANO_SRC_KMODEL, NANO_HEAD_KMODEL, thresh=NANO_THRESH)
print("[NANO] OK")

fc = 0; fps = 0; t0 = time.time()
last_seq = -1     # frame kamera terakhir yang sudah diproses

# State lock: NanoTrack + filter alpha-beta (cx,cy,vx,vy) + status taktis.
track = {"active": False, "cls": None, "w": 0.0, "h": 0.0,
         "cx": 0.0, "cy": 0.0, "vx": 0.0, "vy": 0.0,
         "score": 0.0, "miss": 0, "status": "SCAN"}

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
    inf_ms = 0.0
    ndet = 0
    lock_info = {"active": False, "lost": False}

    if not requested:
        track["active"] = False

    # ============ TRACK: NanoTrack + filter alpha-beta + auto re-lock =
    if track["active"]:
        px = track["cx"] + track["vx"]      # prediksi posisi
        py = track["cy"] + track["vy"]
        t_inf = time.time()
        ok, (bx, by, bw, bh), score = tracker.update(frame)   # KPU
        inf_ms = (time.time() - t_inf) * 1000
        track["score"] = score
        if ok:
            mcx = bx + bw/2.0; mcy = by + bh/2.0
            rx = mcx - px; ry = mcy - py     # koreksi (alpha-beta)
            track["cx"] = px + ALPHA*rx; track["cy"] = py + ALPHA*ry
            track["vx"] += BETA*rx;      track["vy"] += BETA*ry
            track["w"] = bw; track["h"] = bh
            track["miss"] = 0
            track["status"] = "LOCKED"
        else:
            track["miss"] += 1
            # COAST: pakai prediksi (clamp + redam velocity biar tak ngelantur)
            track["cx"] = min(max(px, 0.0), float(CAM_W))
            track["cy"] = min(max(py, 0.0), float(CAM_H))
            track["vx"] *= 0.9; track["vy"] *= 0.9
            track["status"] = "SEARCH"
            # AUTO RE-LOCK: cari target SEKELAS dekat prediksi via YOLO
            if track["miss"] >= RELOCK_AFTER and (track["miss"] % RELOCK_EVERY == 0):
                dets = yolo_detect(frame); ndet = len(dets)
                best = None; bestd = float(RELOCK_RADIUS)
                for d in dets:
                    if d["cls"] != track["cls"]:
                        continue
                    dd = ((d["ccx"]-track["cx"])**2 + (d["ccy"]-track["cy"])**2) ** 0.5
                    if dd < bestd:
                        bestd = dd; best = d
                if best is not None:
                    try:
                        tracker.init(frame, (best["x1"], best["y1"], best["w"], best["h"]))
                        track["cx"] = best["ccx"]; track["cy"] = best["ccy"]
                        track["w"] = best["w"]; track["h"] = best["h"]
                        track["vx"] = 0.0; track["vy"] = 0.0
                        track["miss"] = 0; track["status"] = "RE-LOCK"
                    except Exception:
                        pass
        fb = (track["cx"] - track["w"]/2.0, track["cy"] - track["h"]/2.0, track["w"], track["h"])
        draw_tactical(display, fb, track["vx"], track["vy"], track["score"],
                      track["status"], cxc, cyc, icx, icy)
        nm = CLASSES[track["cls"]] if (track["cls"] is not None and track["cls"] < len(CLASSES)) else "target"
        lock_info = {"active": True, "lost": track["status"] != "LOCKED", "cls": nm,
                     "x": int(track["cx"]), "y": int(track["cy"]),
                     "dx": int(track["cx"]-cxc), "dy": int(track["cy"]-cyc),
                     "score": round(float(track["score"]), 2), "status": track["status"]}

    # ============ SCAN: YOLO (KPU) + acquire =========================
    else:
        t_inf = time.time()
        dets = yolo_detect(frame)
        inf_ms = (time.time() - t_inf) * 1000
        ndet = len(dets)
        for d in dets:
            cv2.rectangle(display, (d["x1"], d["y1"]), (d["x1"]+d["w"], d["y1"]+d["h"]), (0,255,0), 1)
            cv2.putText(display, "%s %.2f" % (d["nama"], d["s"]), (d["x1"], max(d["y1"]-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            objs.append({"x":d["x1"], "y":d["y1"], "w":d["w"], "h":d["h"], "s":round(d["s"],2), "nama":d["nama"]})
        if requested and dets:
            d0 = min(dets, key=lambda d: (d["ccx"]-cxc)**2 + (d["ccy"]-cyc)**2)
            try:
                tracker.init(frame, (d0["x1"], d0["y1"], d0["w"], d0["h"]))
                track.update(active=True, cls=d0["cls"], cx=d0["ccx"], cy=d0["ccy"],
                             vx=0.0, vy=0.0, w=d0["w"], h=d0["h"], score=1.0,
                             miss=0, status="LOCKED")
            except Exception as e:
                print("[NANO] init error:", e)
                track["active"] = False

    # Crosshair tengah
    cv2.line(display, (icx-15, icy), (icx+15, icy), (255, 255, 0), 1)
    cv2.line(display, (icx, icy-15), (icx, icy+15), (255, 255, 0), 1)

    mode_txt = "LOCK" if track["active"] else "SCAN"
    cv2.putText(display, "%s | %.1f FPS | KPU: %.0f ms" % (mode_txt, fps, inf_ms), (5, 20),
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
