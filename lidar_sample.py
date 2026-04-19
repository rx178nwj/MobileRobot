#!/usr/bin/env python3
"""
LiDAR 計測サンプル
==================
YDLidar SDK を直接使ってスキャンデータを取得するサンプルです。

Usage:
  # テキスト出力のみ (3スキャン)
  python3 lidar_sample.py

  # リアルタイム極座標グラフ表示 (Ctrl+C で停止)
  python3 lidar_sample.py --plot

  # スキャン回数を指定 (0 = 無限)
  python3 lidar_sample.py --scans 5

  # ROS2 /scan トピックモード
  python3 lidar_sample.py --ros2 [--plot]
"""

import argparse
import math
import sys
import threading
import time


# ─────────────────────────────────────────────
#  LiDAR パラメータ (YDLidar Tmini Plus, モデル151)
# ─────────────────────────────────────────────
LIDAR_PORT      = "/dev/ttyUSB0"
LIDAR_BAUDRATE  = 230400
LIDAR_FREQ_HZ   = 10.0
LIDAR_SAMPLE_KHZ = 4
LIDAR_RANGE_MAX  = 12.0   # m
LIDAR_RANGE_MIN  = 0.03   # m


# ─────────────────────────────────────────────
#  共通ユーティリティ
# ─────────────────────────────────────────────

def format_scan_summary(points: list[tuple[float, float]]) -> str:
    valid = [(a, r) for a, r in points if r > 0.0]
    if not valid:
        return "  有効点なし"

    angles = [a for a, _ in valid]
    ranges = [r for _, r in valid]
    nearest_idx  = ranges.index(min(ranges))
    farthest_idx = ranges.index(max(ranges))

    return "\n".join([
        f"  総点数   : {len(points)} 点  (有効: {len(valid)} 点)",
        f"  角度範囲 : {min(angles):.1f}° 〜 {max(angles):.1f}°",
        f"  最近傍   : {ranges[nearest_idx]:.3f} m  @ {angles[nearest_idx]:.1f}°",
        f"  最遠点   : {ranges[farthest_idx]:.3f} m  @ {angles[farthest_idx]:.1f}°",
        f"  平均距離 : {sum(ranges)/len(ranges):.3f} m",
    ])


def print_sector_view(points: list[tuple[float, float]], resolution: int = 8) -> None:
    sector_size = 360 / resolution
    buckets: dict[int, list[float]] = {i: [] for i in range(resolution)}
    for angle, rng in points:
        if rng <= 0.0:
            continue
        idx = int((angle % 360) / sector_size) % resolution
        buckets[idx].append(rng)

    labels = {
        0: "前  (  0°)", 1: "右前( 45°)", 2: "右  ( 90°)", 3: "右後(135°)",
        4: "後  (180°)", 5: "左後(225°)", 6: "左  (270°)", 7: "左前(315°)",
    }
    print("  ── セクター最近傍 ──")
    for i in range(resolution):
        label = labels.get(i, f"{i * sector_size:.0f}°")
        if buckets[i]:
            nearest = min(buckets[i])
            bar = "█" * min(max(1, int(nearest * 10)), 30)
            print(f"  {label} : {nearest:5.2f} m  {bar}")
        else:
            print(f"  {label} : ---")


# ─────────────────────────────────────────────
#  LiDAR 初期化ヘルパー
# ─────────────────────────────────────────────

def create_laser(port: str, baudrate: int):
    """CYdLidar を生成してパラメータを設定して返す。"""
    import ydlidar

    ydlidar.os_init()

    ports = ydlidar.lidarPortList()
    usb_ports = [v for v in ports.values() if "ttyUSB" in v]
    if usb_ports:
        port = usb_ports[0]
        print(f"[INFO] 検出ポート: {port}")
    elif ports:
        port = list(ports.values())[0]
        print(f"[INFO] 検出ポート (USB 以外): {port}")
    else:
        print(f"[INFO] 自動検出失敗 — 指定ポートを使用: {port}")

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort,     port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
    laser.setlidaropt(ydlidar.LidarPropLidarType,      ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType,     ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency,  LIDAR_FREQ_HZ)
    laser.setlidaropt(ydlidar.LidarPropSampleRate,     LIDAR_SAMPLE_KHZ)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel,  False)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy,      True)
    laser.setlidaropt(ydlidar.LidarPropIntenstiyBit,   8)
    laser.setlidaropt(ydlidar.LidarPropMaxRange,       LIDAR_RANGE_MAX)
    laser.setlidaropt(ydlidar.LidarPropMinRange,       LIDAR_RANGE_MIN)
    laser.setlidaropt(ydlidar.LidarPropReversion,      True)
    laser.setlidaropt(ydlidar.LidarPropInverted,       True)
    return laser


# ─────────────────────────────────────────────
#  1. テキストモード
# ─────────────────────────────────────────────

def run_sdk_mode(port: str, baudrate: int, num_scans: int) -> None:
    try:
        import ydlidar
    except ImportError:
        print("[ERROR] ydlidar モジュールが見つかりません。")
        sys.exit(1)

    laser = create_laser(port, baudrate)
    print(f"[INFO] LiDAR 初期化中 ...")
    if not laser.initialize():
        print("[ERROR] 初期化失敗")
        laser.disconnecting()
        sys.exit(1)

    print("[INFO] LiDAR 起動中 ...")
    if not laser.turnOn():
        print("[ERROR] 起動失敗")
        laser.disconnecting()
        sys.exit(1)

    scan = ydlidar.LaserScan()
    scan_count = 0
    sep = "─" * 58

    try:
        while ydlidar.os_isOk():
            if num_scans > 0 and scan_count >= num_scans:
                break
            if not laser.doProcessSimple(scan):
                time.sleep(0.1)
                continue

            scan_count += 1
            hz = 1.0 / scan.config.scan_time if scan.config.scan_time > 0 else 0
            points = [(math.degrees(p.angle), p.range) for p in scan.points]

            print(f"\n{sep}")
            print(f"  スキャン #{scan_count}  周波数: {hz:.1f} Hz")
            print(sep)
            print(format_scan_summary(points))
            print()
            print_sector_view(points)

            front = [r for a, r in points if r > 0.0 and (a <= 15.0 or a >= 345.0)]
            if front:
                d = min(front)
                warn = "  ⚠ 接近!" if d < 0.3 else ""
                print(f"\n  前方最近傍 : {d:.3f} m{warn}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C — 停止します")
    finally:
        laser.turnOff()
        laser.disconnecting()
        print(f"[INFO] 完了 (取得スキャン数: {scan_count})")


# ─────────────────────────────────────────────
#  2. グラフモード (リアルタイム極座標プロット)
# ─────────────────────────────────────────────

def run_plot_mode(port: str, baudrate: int) -> None:
    """
    LiDAR スキャンをリアルタイムで極座標グラフ表示する。

    - LiDAR スレッド: 別スレッドでスキャンを取得し shared_state に書き込む
    - メインスレッド: matplotlib アニメーションで ~10fps 更新
    """
    def _detect_display() -> bool:
        """実際に Tk ウィンドウを生成できるか試してディスプレイ有無を判定する。"""
        try:
            import tkinter
            root = tkinter.Tk()
            root.destroy()
            return True
        except Exception:
            return False

    try:
        import ydlidar
        import numpy as np
        import matplotlib
    except ImportError as e:
        print(f"[ERROR] {e}")
        print("  pip3 install matplotlib numpy --break-system-packages")
        sys.exit(1)

    has_display = _detect_display()
    matplotlib.use("TkAgg" if has_display else "Agg")

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable

    if not has_display:
        print("[INFO] ディスプレイ未接続 — PNG ファイル保存モードで動作します")
        print("[INFO]   DISPLAY=:0 が有効な環境で実行するとウィンドウ表示になります")

    # ── スレッド間共有データ ──────────────────────────────────────
    shared_state: dict = {
        "angles_rad": np.array([]),
        "ranges":     np.array([]),
        "scan_count": 0,
        "hz":         0.0,
        "lock":       threading.Lock(),
        "running":    True,
    }

    # ── LiDAR スレッド ────────────────────────────────────────────
    def lidar_thread() -> None:
        laser = create_laser(port, baudrate)
        print("[INFO] LiDAR 初期化中 ...")
        if not laser.initialize():
            print("[ERROR] LiDAR 初期化失敗")
            shared_state["running"] = False
            laser.disconnecting()
            return

        print("[INFO] LiDAR 起動中 ...")
        if not laser.turnOn():
            print("[ERROR] LiDAR 起動失敗")
            shared_state["running"] = False
            laser.disconnecting()
            return

        scan = ydlidar.LaserScan()
        try:
            while shared_state["running"] and ydlidar.os_isOk():
                if not laser.doProcessSimple(scan):
                    time.sleep(0.05)
                    continue

                hz = 1.0 / scan.config.scan_time if scan.config.scan_time > 0 else 0.0

                # 有効点のみ抽出
                angles_rad = []
                ranges     = []
                for p in scan.points:
                    if LIDAR_RANGE_MIN <= p.range <= LIDAR_RANGE_MAX:
                        angles_rad.append(p.angle)
                        ranges.append(p.range)

                with shared_state["lock"]:
                    shared_state["angles_rad"] = np.array(angles_rad)
                    shared_state["ranges"]     = np.array(ranges)
                    shared_state["scan_count"] += 1
                    shared_state["hz"]         = hz
        finally:
            laser.turnOff()
            laser.disconnecting()

    thread = threading.Thread(target=lidar_thread, daemon=True)
    thread.start()

    # LiDAR 起動待ち
    print("[INFO] グラフ起動待ち ...")
    for _ in range(50):
        if not shared_state["running"]:
            print("[ERROR] LiDAR スレッドが起動に失敗しました")
            sys.exit(1)
        with shared_state["lock"]:
            if shared_state["scan_count"] > 0:
                break
        time.sleep(0.1)

    # ── matplotlib 極座標プロット設定 ─────────────────────────────
    plt.rcParams["font.family"] = "Noto Sans CJK JP"  # 日本語フォント指定

    fig = plt.figure(figsize=(8, 8), facecolor="#1a1a2e")
    ax  = fig.add_subplot(111, projection="polar", facecolor="#16213e")

    ax.set_theta_zero_location("N")   # 0° を上 (前方) に
    ax.set_theta_direction(-1)        # 時計回り
    ax.set_ylim(0, LIDAR_RANGE_MAX)
    ax.set_yticks([1, 2, 3, 4, 6, 8, 10, 12])
    ax.tick_params(colors="#aaaacc")
    ax.grid(color="#334466", linestyle="--", linewidth=0.5, alpha=0.7)

    # 同心円ラベルの色
    for label in ax.get_yticklabels():
        label.set_color("#aaaacc")
        label.set_fontsize(8)

    # 方位ラベル — set_xticks() で位置を固定してから set_xticklabels() を呼ぶ
    ax.set_xticks(np.linspace(0, 2 * np.pi, 8, endpoint=False))
    ax.set_xticklabels(
        ["前\n0°", "45°", "右\n90°", "135°", "後\n180°", "225°", "左\n270°", "315°"],
        color="#ccccee", fontsize=9,
    )

    # カラーマップ: 近い = 赤、遠い = 青
    cmap = plt.get_cmap("RdYlGn")
    norm = Normalize(vmin=0, vmax=LIDAR_RANGE_MAX)

    scatter = ax.scatter([], [], s=3, c=[], cmap=cmap, norm=norm, alpha=0.85)

    # 前方警戒ライン (±15°)
    warn_angles = np.linspace(-math.pi * 15 / 180, math.pi * 15 / 180, 50)
    ax.fill_between(warn_angles, 0, 0.3, color="red", alpha=0.15)

    # カラーバー
    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.1, shrink=0.6)
    cbar.set_label("距離 (m)", color="#ccccee")
    cbar.ax.yaxis.set_tick_params(color="#ccccee")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="#ccccee")

    title = ax.set_title("LiDAR スキャン", color="#eeeeff", fontsize=13, pad=15)

    status_text = ax.text(
        0.01, 0.99, "", transform=ax.transAxes,
        color="#88ff88", fontsize=9, verticalalignment="top",
        fontfamily="Noto Sans CJK JP",
    )

    # ── アニメーション更新関数 ────────────────────────────────────
    def update(_frame):
        with shared_state["lock"]:
            angles = shared_state["angles_rad"].copy()
            ranges = shared_state["ranges"].copy()
            count  = shared_state["scan_count"]
            hz     = shared_state["hz"]

        if len(angles) == 0:
            return scatter, status_text

        scatter.set_offsets(np.column_stack([angles, ranges]))
        scatter.set_array(ranges)

        # 前方最近傍
        front_mask = (np.abs(angles) <= math.radians(15)) & (ranges > 0)
        if front_mask.any():
            front_min = ranges[front_mask].min()
            warn = " ⚠ 接近!" if front_min < 0.3 else ""
            front_str = f"前方: {front_min:.2f} m{warn}"
        else:
            front_str = "前方: ---"

        status_text.set_text(
            f"スキャン #{count}  {hz:.1f} Hz\n"
            f"点数: {len(ranges)}\n"
            f"{front_str}"
        )
        title.set_text(f"LiDAR スキャン  #{count}")

        return scatter, status_text

    plt.tight_layout()

    if has_display:
        # FuncAnimation は Xvfb 上で描画がトリガーされないため
        # plt.pause() ループで明示的に draw() する方式を使用
        plt.ion()
        plt.show()
        print("[INFO] グラフ表示中 — ウィンドウを閉じるか Ctrl+C で停止", flush=True)
        try:
            while shared_state["running"] and plt.fignum_exists(fig.number):
                update(None)
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.1)
        except KeyboardInterrupt:
            pass
    else:
        # ヘッドレス: スキャンごとに PNG を上書き保存
        save_path = "/tmp/lidar_scan.png"
        print(f"[INFO] PNG 保存モード: {save_path}  (Ctrl+C で停止)", flush=True)
        last_saved = -1
        try:
            while shared_state["running"]:
                with shared_state["lock"]:
                    count = shared_state["scan_count"]
                if count != last_saved and count > 0:
                    update(None)
                    fig.savefig(save_path, dpi=120, facecolor=fig.get_facecolor())
                    last_saved = count
                    print(f"\r[INFO] スキャン #{count} → {save_path}", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()

    shared_state["running"] = False
    thread.join(timeout=3.0)
    print(f"\n[INFO] 完了 (取得スキャン数: {shared_state['scan_count']})")


# ─────────────────────────────────────────────
#  3. ROS2 トピックモード
# ─────────────────────────────────────────────

def run_ros2_mode(num_scans: int, plot: bool) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
    except ImportError:
        print("[ERROR] rclpy が見つかりません。")
        print("  source /opt/ros/jazzy/setup.bash")
        sys.exit(1)

    if plot:
        # グラフモード: ROS2 ノードをスレッドで動かし、プロットはメインスレッド
        try:
            import numpy as np
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            import matplotlib.animation as animation
            from matplotlib.colors import Normalize
            from matplotlib.cm import ScalarMappable
        except ImportError as e:
            print(f"[ERROR] {e}")
            sys.exit(1)

        shared_state: dict = {
            "angles_rad": np.array([]),
            "ranges":     np.array([]),
            "scan_count": 0,
            "hz":         0.0,
            "lock":       threading.Lock(),
        }

        rclpy.init()

        class LidarSubscriber(Node):
            def __init__(self):
                super().__init__("lidar_sample")
                self.create_subscription(LaserScan, "/scan", self._cb, 10)
                self.get_logger().info("/scan 待機中 ...")

            def _cb(self, msg: LaserScan):
                hz = 1.0 / msg.scan_time if msg.scan_time > 0 else 0.0
                angles, ranges = [], []
                for i, r in enumerate(msg.ranges):
                    if msg.range_min <= r <= msg.range_max:
                        angles.append(msg.angle_min + i * msg.angle_increment)
                        ranges.append(r)
                with shared_state["lock"]:
                    shared_state["angles_rad"] = np.array(angles)
                    shared_state["ranges"]     = np.array(ranges)
                    shared_state["scan_count"] += 1
                    shared_state["hz"]         = hz

        node = LidarSubscriber()
        ros_thread = threading.Thread(
            target=lambda: rclpy.spin(node), daemon=True
        )
        ros_thread.start()

        # プロット部分は run_plot_mode と同じ構造を再利用
        plt.rcParams["font.family"] = "Noto Sans CJK JP"

        fig = plt.figure(figsize=(8, 8), facecolor="#1a1a2e")
        ax  = fig.add_subplot(111, projection="polar", facecolor="#16213e")
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_ylim(0, LIDAR_RANGE_MAX)
        ax.tick_params(colors="#aaaacc")
        ax.grid(color="#334466", linestyle="--", linewidth=0.5, alpha=0.7)
        ax.set_xticks(np.linspace(0, 2 * np.pi, 8, endpoint=False))
        ax.set_xticklabels(
            ["前\n0°", "45°", "右\n90°", "135°", "後\n180°", "225°", "左\n270°", "315°"],
            color="#ccccee", fontsize=9,
        )
        cmap  = plt.get_cmap("RdYlGn")
        norm  = Normalize(vmin=0, vmax=LIDAR_RANGE_MAX)
        scatter = ax.scatter([], [], s=3, c=[], cmap=cmap, norm=norm, alpha=0.85)
        title   = ax.set_title("LiDAR スキャン (ROS2)", color="#eeeeff", fontsize=13, pad=15)
        status_text = ax.text(
            0.01, 0.99, "", transform=ax.transAxes,
            color="#88ff88", fontsize=9, va="top", fontfamily="monospace",
        )

        def update(_frame):
            with shared_state["lock"]:
                angles = shared_state["angles_rad"].copy()
                ranges = shared_state["ranges"].copy()
                count  = shared_state["scan_count"]
                hz     = shared_state["hz"]
            if len(angles) == 0:
                return scatter, status_text
            scatter.set_offsets(np.column_stack([angles, ranges]))
            scatter.set_array(ranges)
            status_text.set_text(f"スキャン #{count}  {hz:.1f} Hz\n点数: {len(ranges)}")
            title.set_text(f"LiDAR スキャン (ROS2)  #{count}")
            return scatter, status_text

        ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        print("[INFO] グラフ表示中 — ウィンドウを閉じるか Ctrl+C で停止")
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()
            print(f"[INFO] 完了 (取得スキャン数: {shared_state['scan_count']})")
        return

    # テキストモード
    class LidarSubscriber(Node):
        def __init__(self):
            super().__init__("lidar_sample")
            self.scan_count = 0
            self.create_subscription(LaserScan, "/scan", self._cb, 10)
            self.get_logger().info("/scan 待機中 ...")

        def _cb(self, msg: LaserScan):
            if num_scans > 0 and self.scan_count >= num_scans:
                rclpy.shutdown()
                return
            self.scan_count += 1
            hz = 1.0 / msg.scan_time if msg.scan_time > 0 else 0.0
            points = []
            for i, r in enumerate(msg.ranges):
                a = math.degrees(msg.angle_min + i * msg.angle_increment)
                points.append((a, r if msg.range_min <= r <= msg.range_max else 0.0))
            sep = "─" * 58
            print(f"\n{sep}")
            print(f"  スキャン #{self.scan_count}  frame: {msg.header.frame_id}  {hz:.1f} Hz")
            print(sep)
            print(format_scan_summary(points))
            print()
            print_sector_view(points)

    rclpy.init()
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C — 停止します")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[INFO] 完了 (取得スキャン数: {node.scan_count})")


# ─────────────────────────────────────────────
#  エントリーポイント
# ─────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="YDLidar 計測サンプル",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--ros2",     action="store_true", help="ROS2 /scan トピックモード")
    parser.add_argument("--plot",     action="store_true", help="リアルタイム極座標グラフ表示")
    parser.add_argument("--port",     default=LIDAR_PORT,     help=f"シリアルポート (デフォルト: {LIDAR_PORT})")
    parser.add_argument("--baudrate", type=int, default=LIDAR_BAUDRATE, help=f"ボーレート (デフォルト: {LIDAR_BAUDRATE})")
    parser.add_argument("--scans",    type=int, default=3,    help="取得スキャン回数 (0=無限, デフォルト: 3, --plot 時は無視)")
    args = parser.parse_args()

    if args.ros2:
        print("[MODE] ROS2 トピックモード" + (" + グラフ" if args.plot else ""))
        run_ros2_mode(num_scans=args.scans, plot=args.plot)
    elif args.plot:
        print("[MODE] SDK 直接モード + リアルタイムグラフ")
        run_plot_mode(port=args.port, baudrate=args.baudrate)
    else:
        print("[MODE] SDK 直接モード")
        run_sdk_mode(port=args.port, baudrate=args.baudrate, num_scans=args.scans)


if __name__ == "__main__":
    main()
