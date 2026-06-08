#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test minimal: embed libmpv ke window PyQt5.
Tujuan: cek apakah mpv embedding jalan di sistem ini, lepas dari app besar.

Jalankan:
    python3 test_mpv.py
atau dengan stream:
    python3 test_mpv.py udp://127.0.0.1:5600
"""
import sys
import locale

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt

import mpv


class Player(QWidget):
    def __init__(self, url=None):
        super().__init__()
        self.resize(640, 480)
        self.setWindowTitle("MPV embed test")

        self.frame = QWidget(self)
        self.frame.setAttribute(Qt.WA_DontCreateNativeAncestors)
        self.frame.setAttribute(Qt.WA_NativeWindow)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.frame)

        self.url = url

    def showEvent(self, e):
        super().showEvent(e)
        if getattr(self, "_mpv", None) is not None:
            return
        # WAJIB sebelum init mpv
        locale.setlocale(locale.LC_NUMERIC, "C")
        print(">> creating mpv on wid =", int(self.frame.winId()))
        self._mpv = mpv.MPV(
            wid=str(int(self.frame.winId())),
            vo="gpu",
            hwdec="no",
            profile="low-latency",
            osc=False,
            border=False,
            idle="yes",
            force_window="yes",
            terminal=True,          # tampilkan log mpv di terminal untuk debug
            msg_level="all=info",
        )
        self._mpv["demuxer-lavf-o"] = "fflags=+nobuffer+discardcorrupt"
        self._mpv["untimed"] = True
        print(">> mpv created OK")
        if self.url:
            print(">> playing", self.url)
            self._mpv.play(self.url)


def main():
    app = QApplication(sys.argv)
    locale.setlocale(locale.LC_NUMERIC, "C")
    url = sys.argv[1] if len(sys.argv) > 1 else None
    w = Player(url)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
