#!/usr/bin/env bash
# LiDAR グラフ表示スクリプト (VNC + feh)
# ========================================
# 構成:
#   1. Xvfb  — 仮想ディスプレイ (:1)
#   2. x11vnc — VNC でその画面を公開 (ポート 5900)
#   3. lidar_sample.py — Agg で PNG を /tmp/lidar_scan.png に書き出し
#   4. feh   — PNG をウィンドウで自動更新表示 (VNC 越しに見える)
#
# 使い方:
#   ./lider.sh          — 起動 (Ctrl+C で停止)
#   ./lider.sh --stop   — バックグラウンドプロセスを停止

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAMPLE="${SCRIPT_DIR}/lidar_sample.py"
PNG_PATH="/tmp/lidar_scan.png"

VNC_DISPLAY=":1"
VNC_PORT="5900"
VNC_RESOLUTION="900x900x24"

XVFB_PID_FILE="/tmp/lidar_xvfb.pid"
VNC_PID_FILE="/tmp/lidar_vnc.pid"
VIEWER_PID_FILE="/tmp/lidar_viewer.pid"

SEP="────────────────────────────────────────────"

# ── 停止 ─────────────────────────────────────────────────────────
stop_all() {
    echo "${SEP}"
    echo "  停止中..."
    for pidfile in "${VIEWER_PID_FILE}" "${VNC_PID_FILE}" "${XVFB_PID_FILE}"; do
        if [ -f "${pidfile}" ]; then
            pid=$(cat "${pidfile}")
            kill "${pid}" 2>/dev/null && echo "  停止: PID ${pid}" || true
            rm -f "${pidfile}"
        fi
    done
    pkill -f "lidar_sample.py" 2>/dev/null || true
    echo "  完了"
    echo "${SEP}"
}

if [ "${1}" = "--stop" ]; then
    stop_all
    exit 0
fi

# ── Xvfb 起動 ────────────────────────────────────────────────────
if [ -f "${XVFB_PID_FILE}" ] && kill -0 "$(cat ${XVFB_PID_FILE})" 2>/dev/null; then
    echo "  Xvfb 起動済み"
else
    echo "  Xvfb 起動中..."
    rm -f /tmp/.X1-lock
    Xvfb "${VNC_DISPLAY}" -screen 0 "${VNC_RESOLUTION}" -ac &
    echo $! > "${XVFB_PID_FILE}"
    sleep 1
    DISPLAY="${VNC_DISPLAY}" openbox --sm-disable 2>/dev/null &
    sleep 0.5
fi

# ── x11vnc 起動 ───────────────────────────────────────────────────
if [ -f "${VNC_PID_FILE}" ] && kill -0 "$(cat ${VNC_PID_FILE})" 2>/dev/null; then
    echo "  x11vnc 起動済み"
else
    echo "  x11vnc 起動中 (ポート ${VNC_PORT})..."
    x11vnc -display "${VNC_DISPLAY}" -nopw -rfbport "${VNC_PORT}" \
           -forever -quiet -bg -o /tmp/lidar_vnc.log
    sleep 0.5
    pgrep -n x11vnc > "${VNC_PID_FILE}" 2>/dev/null || true
fi

# ── 初期 PNG 生成 (feh が最初に読むファイルが必要) ───────────────
if [ ! -f "${PNG_PATH}" ]; then
    python3 - <<'EOF'
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "Noto Sans CJK JP"
fig = plt.figure(figsize=(9, 9), facecolor="#1a1a2e")
ax = fig.add_subplot(111, projection="polar", facecolor="#16213e")
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.text(0, 0, "LiDAR 起動中...", ha="center", va="center",
        color="white", fontsize=16, transform=ax.transData)
fig.savefig("/tmp/lidar_scan.png", dpi=120, facecolor="#1a1a2e")
EOF
fi

# ── PNG ビューア起動 (Tkinter, 500ms 自動更新) ───────────────────
if [ -f "${VIEWER_PID_FILE}" ] && kill -0 "$(cat ${VIEWER_PID_FILE})" 2>/dev/null; then
    echo "  ビューア起動済み"
else
    echo "  ビューア起動中..."
    DISPLAY="${VNC_DISPLAY}" python3 "${SCRIPT_DIR}/lidar_viewer.py" &
    echo $! > "${VIEWER_PID_FILE}"
    sleep 1
fi

# ── 接続情報表示 ─────────────────────────────────────────────────
PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "${SEP}"
echo "  VNC 接続先: ${PI_IP}:${VNC_PORT}  (パスワードなし)"
echo ""
echo "  Ubuntu:"
echo "    sudo apt install tigervnc-viewer"
echo "    vncviewer ${PI_IP}:${VNC_PORT}"
echo "${SEP}"
echo ""
echo "  LiDAR 起動中... (Ctrl+C で停止)"
echo ""

# ── lidar_sample.py を PNG 保存モード (Agg) で実行 ───────────────
# DISPLAY を未設定にして Agg モード強制
unset DISPLAY
exec python3 "${SAMPLE}" --plot "$@"
