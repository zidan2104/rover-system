#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Profil NanoTrack: ukur waktu tiap tahap (sub_window/prep/src-KPU/head-KPU/decode).
Jalankan di K230:  cd /root && python3 nano_profile.py
"""
import time, numpy as np, cv2
from nanotrack_kpu import (NanoTrackKPU, _sub_window,
                           EXEMPLAR_SIZE, INSTANCE_SIZE, CONTEXT_AMOUNT, OUTPUT_SIZE)

CROP = "/root/cropped_test127.kmodel"
SRC  = "/root/nanotrack_backbone_sim.kmodel"
HEAD = "/root/nanotracker_head_calib_k230.kmodel"

tr = NanoTrackKPU(CROP, SRC, HEAD)
img = np.zeros((480, 640, 3), np.uint8)
cv2.rectangle(img, (280, 200), (360, 280), (128, 200, 90), -1)
tr.init(img, (280, 200, 80, 80))

# warmup (run pertama KPU biasanya lebih lambat)
for _ in range(3):
    tr.update(img)

N = 30
acc = [0.0, 0.0, 0.0, 0.0, 0.0]
rows, cols = img.shape[:2]
for _ in range(N):
    w0, h0 = tr.rect_size
    w_z = w0 + CONTEXT_AMOUNT * (w0 + h0)
    h_z = h0 + CONTEXT_AMOUNT * (w0 + h0)
    s_z = round((w_z * h_z) ** 0.5)
    s_x = round(s_z * INSTANCE_SIZE / EXEMPLAR_SIZE)

    t0 = time.time(); patch = _sub_window(img, INSTANCE_SIZE, s_x, tr.center)
    t1 = time.time(); inp = tr._prep(patch)
    t2 = time.time(); sf = tr.src.run1(inp)
    t3 = time.time(); o0, o1 = tr.head.run2(tr.template_feat,
                                            np.ascontiguousarray(sf.astype(np.float32)))
    t4 = time.time()
    a = o0.reshape(-1).astype(np.float64); b = o1.reshape(-1).astype(np.float64)
    score, box = (a, b) if a.size == 2 * OUTPUT_SIZE else (b, a)
    tr._decode(score, box, cols, rows)
    t5 = time.time()

    acc[0] += t1 - t0; acc[1] += t2 - t1; acc[2] += t3 - t2
    acc[3] += t4 - t3; acc[4] += t5 - t4

acc = [x / N * 1000 for x in acc]
tot = sum(acc)
print("==== NanoTrack profil (rata-rata %d frame) ====" % N)
print("sub_window (crop+resize) : %6.2f ms" % acc[0])
print("prep (transpose+copy)    : %6.2f ms" % acc[1])
print("src  KPU  (backbone 255) : %6.2f ms" % acc[2])
print("head KPU                 : %6.2f ms" % acc[3])
print("decode (numpy)           : %6.2f ms" % acc[4])
print("-----------------------------------------------")
print("TOTAL                    : %6.2f ms  (%.1f fps)" % (tot, 1000.0 / tot))
print("KPU total                : %6.2f ms  (%.0f%%)" % (acc[2] + acc[3],
                                                         (acc[2] + acc[3]) / tot * 100))
print("Glue CPU total           : %6.2f ms  (%.0f%%)" % (acc[0] + acc[1] + acc[4],
                                                         (acc[0] + acc[1] + acc[4]) / tot * 100))
