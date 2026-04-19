#!/usr/bin/env python3
"""
LiDAR PNG ビューア
==================
/tmp/lidar_scan.png を 500ms ごとに自動更新して Tk ウィンドウに表示する。
lidar_sample.py --plot (Agg PNG 保存モード) と組み合わせて使う。

Usage:
  DISPLAY=:1 python3 lidar_viewer.py
"""

import sys
import tkinter as tk
from pathlib import Path
from PIL import Image, ImageTk

PNG_PATH = Path("/tmp/lidar_scan.png")
REFRESH_MS = 500   # 更新間隔 (ms)
WINDOW_SIZE = 850  # ウィンドウサイズ (px)


def main() -> None:
    root = tk.Tk()
    root.title("LiDAR スキャン")
    root.configure(bg="#1a1a2e")
    root.geometry(f"{WINDOW_SIZE}x{WINDOW_SIZE}")
    root.resizable(True, True)

    label = tk.Label(root, bg="#1a1a2e")
    label.pack(fill=tk.BOTH, expand=True)

    photo_ref = [None]  # GC 防止用

    def refresh() -> None:
        try:
            if PNG_PATH.exists():
                img = Image.open(PNG_PATH)
                # ウィンドウサイズに合わせてリサイズ
                w = label.winfo_width() or WINDOW_SIZE
                h = label.winfo_height() or WINDOW_SIZE
                img = img.resize((w, h), Image.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                label.configure(image=photo)
                photo_ref[0] = photo  # GC 防止
        except Exception:
            pass  # 書き込み中は無視して次回リトライ
        root.after(REFRESH_MS, refresh)

    # 初回表示
    root.after(100, refresh)
    root.mainloop()


if __name__ == "__main__":
    main()
