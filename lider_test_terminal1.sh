#!/bin/bash
# 仮想ディスプレイ起動 (1280x800, :1)
Xvfb :1 -screen 0 1280x800x24 &

# VNC でその画面を公開 (パスワードなし、ループバックのみ)
x11vnc -display :1 -nopw -listen localhost -xkb &