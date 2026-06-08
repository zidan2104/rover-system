#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NanoTrack single-object tracker — FULL KPU (K230).
Port dari referensi C++ Canaan (ai_poc/nanotracker: cv2_utils.cc + main.cc).

3 kmodel (semua inferensi di KPU):
  crop  (template @127x127) -> fitur [1,48,8,8]      (dijalankan sekali saat LOCK)
  src   (search   @255x255) -> fitur [1,48,16,16]    (tiap frame)
  head  (2 input fitur)     -> score[2,16,16], box[4,16,16]

Pemakaian:
  tr = NanoTrackKPU(crop_path, src_path, head_path)
  tr.init(frame_bgr, (x, y, w, h))      # saat lock, box dari YOLO
  ok, (x, y, w, h), score = tr.update(frame_bgr)   # tiap frame
"""
import cv2
import numpy as np
import nncaseruntime as nn

# --- Konstanta (dari cv2_utils.cc) ---
EXEMPLAR_SIZE    = 127
INSTANCE_SIZE    = 255
CONTEXT_AMOUNT   = 0.5
OUTPUT_GRID      = 16
OUTPUT_SIZE      = 256          # 16*16
WINDOW_INFLUENCE = 0.46
LR               = 0.34
PENALTY_K        = 0.16

_HANNING = np.array(
    [0., 0.04322727, 0.1654347, 0.3454915, 0.55226423, 0.75, 0.9045085, 0.9890738,
     0.9890738, 0.9045085, 0.75, 0.55226423, 0.3454915, 0.1654347, 0.04322727, 0.],
    dtype=np.float64)


def _make_window():
    # window[i*16+j] = hanning[i]*hanning[j]  -> flatten 256
    return np.outer(_HANNING, _HANNING).reshape(-1)


def _make_points():
    # points[idx] = (x_col, y_row), mulai -128 step 16  (set_points di cv2_utils.cc)
    pts = np.zeros((OUTPUT_SIZE, 2), dtype=np.float64)
    for i in range(OUTPUT_GRID):
        x = -128.0
        for j in range(OUTPUT_GRID):
            pts[i * OUTPUT_GRID + j, 0] = x
            x += OUTPUT_GRID
    y = -128.0
    for i in range(OUTPUT_GRID):
        for j in range(OUTPUT_GRID):
            pts[i * OUTPUT_GRID + j, 1] = y
        y += OUTPUT_GRID
    return pts


def _sub_window(img, size, length, center):
    """Crop kotak sisi `length` berpusat di `center`, pad warna rata, resize ke size.
    OPTIMIZED: pad HANYA area crop (bukan seluruh gambar) + cv2.mean (C-cepat).
    Setara sub_window() C++ tapi jauh lebih ringan di RISC-V."""
    h, w = img.shape[:2]
    s_z = int(round(length))
    c = (s_z + 1) / 2.0
    cx, cy = center
    xmin = int(np.floor(cx - c + 0.5)); ymin = int(np.floor(cy - c + 0.5))
    xmax = xmin + s_z - 1;              ymax = ymin + s_z - 1
    left   = max(0, -xmin);        top    = max(0, -ymin)
    right  = max(0, xmax - (w - 1)); bottom = max(0, ymax - (h - 1))
    # area valid di dalam gambar
    vx1 = max(0, xmin); vy1 = max(0, ymin)
    vx2 = min(w - 1, xmax); vy2 = min(h - 1, ymax)
    crop = img[vy1:vy2 + 1, vx1:vx2 + 1]
    if crop.size == 0:
        return cv2.resize(img, (size, size))
    if left or right or top or bottom:
        m = cv2.mean(img)[:3]      # mean cepat (C), cuma saat perlu pad
        crop = cv2.copyMakeBorder(crop, top, bottom, left, right,
                                  cv2.BORDER_CONSTANT, value=m)
    return cv2.resize(crop, (size, size))


class _KModel:
    """Pembungkus 1 kmodel di KPU lewat nncaseruntime."""
    def __init__(self, path):
        self.ip = nn.Interpreter()
        with open(path, "rb") as f:
            self.ip.load_model(f.read())

    def run1(self, inp):
        self.ip.set_input_tensor(0, nn.RuntimeTensor.from_numpy(inp))
        self.ip.run()
        return self.ip.get_output_tensor(0).to_numpy()

    def run2(self, in0, in1):
        self.ip.set_input_tensor(0, nn.RuntimeTensor.from_numpy(in0))
        self.ip.set_input_tensor(1, nn.RuntimeTensor.from_numpy(in1))
        self.ip.run()
        return (self.ip.get_output_tensor(0).to_numpy(),
                self.ip.get_output_tensor(1).to_numpy())


class NanoTrackKPU:
    def __init__(self, crop_path, src_path, head_path, thresh=0.1, color_rgb=False):
        self.crop = _KModel(crop_path)
        self.src  = _KModel(src_path)
        self.head = _KModel(head_path)
        self.thresh    = thresh
        self.color_rgb = color_rgb          # NanoTrack dilatih RGB; flip kalau hasil buruk
        self.window = _make_window()
        self.points = _make_points()
        self.center = [0.0, 0.0]
        self.rect_size = [0.0, 0.0]
        self.template_feat = None

    def _prep(self, patch):
        img = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB) if self.color_rgb else patch
        return np.ascontiguousarray(
            np.transpose(img, (2, 0, 1))[np.newaxis].astype(np.uint8))

    # --- LOCK: hitung fitur template sekali ---
    def init(self, frame, bbox):
        x, y, w, h = bbox
        self.center    = [x + w / 2.0, y + h / 2.0]
        self.rect_size = [float(w), float(h)]
        w0, h0 = self.rect_size
        w_z = w0 + CONTEXT_AMOUNT * (w0 + h0)
        h_z = h0 + CONTEXT_AMOUNT * (w0 + h0)
        s_z = round(float(np.sqrt(w_z * h_z)))
        patch = _sub_window(frame, EXEMPLAR_SIZE, s_z, self.center)
        feat = self.crop.run1(self._prep(patch))             # [1,48,8,8]
        self.template_feat = np.ascontiguousarray(feat.astype(np.float32))

    # --- tiap frame: search -> head -> decode ---
    def update(self, frame):
        if self.template_feat is None:
            return False, (0, 0, 0, 0), 0.0
        rows, cols = frame.shape[:2]
        w0, h0 = self.rect_size
        w_z = w0 + CONTEXT_AMOUNT * (w0 + h0)
        h_z = h0 + CONTEXT_AMOUNT * (w0 + h0)
        s_z = round(float(np.sqrt(w_z * h_z)))
        s_x = round(s_z * INSTANCE_SIZE / EXEMPLAR_SIZE)
        patch = _sub_window(frame, INSTANCE_SIZE, s_x, self.center)
        src_feat = self.src.run1(self._prep(patch))          # [1,48,16,16]
        o0, o1 = self.head.run2(self.template_feat,
                                np.ascontiguousarray(src_feat.astype(np.float32)))
        a = o0.reshape(-1).astype(np.float64)
        b = o1.reshape(-1).astype(np.float64)
        # auto-deteksi mana score (512) mana box (1024)
        score, box = (a, b) if a.size == 2 * OUTPUT_SIZE else (b, a)
        x, y, bw, bh, sc = self._decode(score, box, cols, rows)
        return (sc > self.thresh), (x, y, bw, bh), sc

    # --- decode: port post_process() C++ ---
    def _decode(self, score, box, cols, rows):
        w0, h0 = self.rect_size
        w_z = w0 + CONTEXT_AMOUNT * (w0 + h0)
        h_z = h0 + CONTEXT_AMOUNT * (w0 + h0)
        s_z = np.sqrt(w_z * h_z)
        scale_z = EXEMPLAR_SIZE / s_z

        # convert_score: softmax atas channel foreground (score[256:512])
        fg = score[OUTPUT_SIZE:2 * OUTPUT_SIZE]
        e = np.exp(fg - fg.max())
        fgp = e / e.sum()

        # convert_bbox: anchor points +- jarak -> cx,cy,w,h
        px = self.points[:, 0]; py = self.points[:, 1]
        x1 = px - box[0:OUTPUT_SIZE]
        y1 = py - box[OUTPUT_SIZE:2 * OUTPUT_SIZE]
        x2 = px + box[2 * OUTPUT_SIZE:3 * OUTPUT_SIZE]
        y2 = py + box[3 * OUTPUT_SIZE:4 * OUTPUT_SIZE]
        cx = (x1 + x2) * 0.5; cy = (y1 + y2) * 0.5
        cw = x2 - x1;         ch = y2 - y1

        # penalty (sz() pakai (w+pad)*(h*pad) — persis seperti C++)
        def _sz(w, h):
            pad = (w + h) * 0.5
            return np.sqrt((w + pad) * (h * pad))
        def _change(r):
            return np.maximum(r, 1.0 / r)

        sz_cur = _sz(cw, ch)
        sz_ref = _sz(w0 * scale_z, h0 * scale_z)
        s_c = _change(sz_cur / sz_ref)
        r_c = _change((w0 / h0) / (cw / ch))
        penalty = np.exp(-(r_c * s_c - 1.0) * PENALTY_K)
        pscore = penalty * fgp
        pscore = pscore * (1 - WINDOW_INFLUENCE) + self.window * WINDOW_INFLUENCE

        best = int(np.argmax(pscore))
        bcx = cx[best] / scale_z; bcy = cy[best] / scale_z
        bcw = cw[best] / scale_z; bch = ch[best] / scale_z
        lr = penalty[best] * fgp[best] * LR

        ncx = bcx + self.center[0]
        ncy = bcy + self.center[1]
        ncw = w0 * (1 - lr) + bcw * lr
        nch = h0 * (1 - lr) + bch * lr
        # clip
        ncx = max(0.0, min(ncx, float(cols)))
        ncy = max(0.0, min(ncy, float(rows)))
        ncw = max(10.0, min(ncw, float(cols)))
        nch = max(10.0, min(nch, float(rows)))
        self.center    = [ncx, ncy]
        self.rect_size = [ncw, nch]

        bscore = float(fgp[best])
        return (max(0, int(ncx - ncw / 2)), max(0, int(ncy - nch / 2)),
                int(ncw), int(nch), bscore)
