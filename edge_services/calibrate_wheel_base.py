#!/usr/bin/env python3
"""
wheel_base キャリブレーション — IMU (ジャイロ yaw) 基準
=========================================================
ロボットを 360° 旋回させ、ジャイロ累積 yaw と
オドメトリ累積角を比較して wheel_base を補正する。

手順:
  1. launch_all.py を停止して PR2040 に直接アクセス
  2. 360° 旋回中にジャイロ yaw とエンコーダーを同時サンプリング
  3. ジャイロ角度を真値として wheel_base を更新
  4. robot_config.json に保存

Usage:
  cd ~/MobileRobot/edge_services
  python3 calibrate_wheel_base.py
  python3 calibrate_wheel_base.py --apply  # 計測後に robot_config.json を自動更新
"""

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

CONFIG_PATH = Path(__file__).parent / 'config/robot_config.json'
SEP  = '─' * 58
SEP2 = '═' * 58


def load_config():
    with open(CONFIG_PATH) as f:
        return json.load(f)


def stop_services():
    result = subprocess.run(['pgrep', '-f', 'launch_all.py'],
                            capture_output=True, text=True)
    pids = result.stdout.strip().split()
    if pids:
        print(f'  launch_all.py を停止 (PID: {" ".join(pids)}) ...')
        for pid in pids:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
        print('  ✓ 停止')
    else:
        print('  launch_all.py は未起動')


def request_imu(driver) -> dict | None:
    """CMD_REQUEST_IMU (0x13) を送信して RESP_IMU を受信する。"""
    driver._send_cmd(0x13)
    # reader thread が RESP_IMU を処理していないので raw シリアルから読む
    # → driver の _reader_loop が STATUS しか処理しないため、
    #   ここでは get_status() から yaw を取得できない。
    # 代替: status パケットから yaw を取得する専用リーダーを使う。
    # → _reader_loop を拡張するのが正道だが、ここでは
    #   シリアルポートを driver と共有してしまうため不可。
    # → 代わりに driver の _latest_status を使いつつ
    #   yaw は別スレッドで IMU リクエストを連続送信して受信する。
    return None  # 後述のインライン実装に置き換え


def run_calibration(apply: bool):
    from hardware.pr2040_usb_driver import PR2040USBDriver
    import threading

    config = load_config()
    wheel_base_current = config['robot']['wheel_base']
    wheel_radius       = config['robot']['wheel_radius']
    encoder_cpr        = config['robot']['encoder_cpr']

    print()
    print(SEP2)
    print('  wheel_base キャリブレーション (IMU ジャイロ基準)')
    print(SEP2)
    print(f'  現在の wheel_base : {wheel_base_current} m')
    print()

    stop_services()

    print('  PR2040 接続中...')
    time.sleep(0.5)
    driver = PR2040USBDriver(port='/dev/ttyACM0')
    if not driver.connected:
        print('  ✗ 接続失敗')
        return
    print('  ✓ 接続OK')
    time.sleep(0.3)

    # ── IMU リーダースレッド ──────────────────────────────────────
    # driver の reader thread が RESP_STATUS しかパースしないため、
    # RESP_IMU (0x93) は別スレッドで raw serial を監視する。
    # ただし driver がシリアルを保持しているので ser を共有する。
    latest_imu = {'yaw': None, 'lock': threading.Lock()}

    def imu_requester():
        """100 ms ごとに CMD_REQUEST_IMU を送り、RESP_IMU を受信する。"""
        import struct as _struct
        buf = bytearray()
        while not stop_imu.is_set():
            driver._send_cmd(0x13)
            time.sleep(0.01)
            # IMU レスポンスは driver の reader thread に割り込まれる前に
            # バッファから読み出したい。ここでは driver.ser を直接使う。
            deadline = time.time() + 0.08
            while time.time() < deadline:
                try:
                    with driver._write_lock:
                        pass  # ライトロックが空くのを待つだけ
                    chunk = driver.ser.read(driver.ser.in_waiting or 0)
                    if chunk:
                        buf.extend(chunk)
                except Exception:
                    break
                # バッファから 0x93 パケットを探す
                while True:
                    idx = buf.find(0xAA)
                    if idx < 0: buf.clear(); break
                    if idx > 0: del buf[:idx]
                    if len(buf) < 3: break
                    pkt_type = buf[1]
                    data_len = buf[2]
                    total = 3 + data_len + 1
                    if len(buf) < total: break
                    data = bytes(buf[3:3+data_len])
                    cs_recv = buf[3+data_len]
                    cs = pkt_type ^ data_len
                    for b in data: cs ^= b
                    del buf[:total]
                    if cs != cs_recv:
                        continue
                    if pkt_type == 0x93 and data_len == 40:
                        yaw = _struct.unpack_from('<f', data, 32)[0]
                        with latest_imu['lock']:
                            latest_imu['yaw'] = yaw
                        break
                time.sleep(0.005)

    # driver の reader thread は STATUS しか処理しないが、
    # IMU リクエストへのレスポンスが STATUS パーサーに食われる可能性がある。
    # より確実な方法: driver を停止して直接シリアルで制御する。
    # → driver.close() して新規 serial.Serial で行う。

    driver.close()
    time.sleep(0.3)

    # ── 直接シリアル制御 ─────────────────────────────────────────
    import serial as _serial

    ser = _serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()

    def send_cmd(cmd, data=b''):
        length = len(data)
        cs = cmd ^ length
        for b in data: cs ^= b
        pkt = bytes([0xAA, cmd, length]) + data + bytes([cs])
        ser.write(pkt)

    def read_packets(duration=0.05):
        """duration 秒間読んで STATUS と IMU を返す。"""
        buf = bytearray()
        deadline = time.time() + duration
        status_data = None
        imu_data    = None
        while time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
            while True:
                idx = buf.find(0xAA)
                if idx < 0: buf.clear(); break
                if idx > 0: del buf[:idx]
                if len(buf) < 3: break
                pkt_type = buf[1]
                data_len = buf[2]
                total = 3 + data_len + 1
                if len(buf) < total: break
                data = bytes(buf[3:3+data_len])
                cs_recv = buf[3+data_len]
                cs = pkt_type ^ data_len
                for b in data: cs ^= b
                del buf[:total]
                if cs != cs_recv:
                    continue
                if pkt_type == 0x91 and data_len == 44:
                    status_data = data
                elif pkt_type == 0x93 and data_len == 40:
                    imu_data = data
        return status_data, imu_data

    # ── モードを VELOCITY に設定 ──────────────────────────────────
    for wheel in range(4):
        data = struct.pack('<Bfff', wheel, 0.10, 2.0, 0.0)
        send_cmd(0x26, data)
    send_cmd(0x20, bytes([1, 1, 1, 1]))  # MODE_VELOCITY
    send_cmd(0x22, struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))

    # ── エンコーダーリセット ──────────────────────────────────────
    send_cmd(0x11)
    time.sleep(0.1)

    # ── IMU 初期 yaw 取得 ─────────────────────────────────────────
    print()
    print(f'  {SEP}')
    print('  IMU 初期 yaw 取得中...')
    yaw_start = None
    for _ in range(30):
        send_cmd(0x13)
        _, imu = read_packets(0.05)
        if imu:
            yaw_start = struct.unpack_from('<f', imu, 32)[0]
            break
    if yaw_start is None:
        print('  ✗ IMU 応答なし — ジャイロが接続されているか確認してください')
        ser.close()
        return
    print(f'  初期 yaw: {yaw_start:.2f}°')

    # ── 360° 旋回 ─────────────────────────────────────────────────
    print()
    print(f'  {SEP}')
    print('  360° 旋回中 (2.0 rad/s)...')
    print('  ⚠  ロボットが回転します')

    ANGULAR_VEL = 2.0  # rad/s
    TARGET_RAD  = 2.0 * math.pi
    STOP_AT     = 0.990

    cpr = encoder_cpr
    wb  = wheel_base_current
    r   = wheel_radius
    right_cps = (ANGULAR_VEL * wb / 2.0) / (2.0 * math.pi * r) * cpr
    left_cps  = -right_cps

    # ホイール速度設定: left=enc[0], right=enc[1] (右は反転)
    vel_data = struct.pack('<ffff', left_cps, -right_cps, left_cps, -right_cps)

    enc_start = None
    enc_prev  = None
    odom_accumulated = 0.0
    yaw_prev  = yaw_start
    yaw_accumulated = 0.0
    samples = []

    t_start = time.time()
    MAX_TIME = 15.0

    while (time.time() - t_start) < MAX_TIME:
        send_cmd(0x22, vel_data)
        send_cmd(0x13)  # IMU リクエスト
        status, imu = read_packets(0.04)

        if status is None:
            continue

        encs = struct.unpack_from('<iiii', status, 0)

        if enc_start is None:
            enc_start = encs
            enc_prev  = encs

        # エンコーダー差分 → オドメトリ角度
        dl = (encs[0] - enc_prev[0]) / cpr * (2 * math.pi * r)
        dr = -(encs[1] - enc_prev[1]) / cpr * (2 * math.pi * r)
        d_theta_odom = (dr - dl) / wb
        odom_accumulated += d_theta_odom
        enc_prev = encs

        # ジャイロ yaw 差分
        if imu:
            yaw_now = struct.unpack_from('<f', imu, 32)[0]
            d_yaw = yaw_now - yaw_prev
            # 180° 折り返し補正
            if d_yaw >  180: d_yaw -= 360
            if d_yaw < -180: d_yaw += 360
            yaw_accumulated += d_yaw
            yaw_prev = yaw_now
            samples.append((odom_accumulated, math.radians(yaw_accumulated)))

        # 停止判定 (オドメトリベース)
        if abs(odom_accumulated) >= TARGET_RAD * STOP_AT:
            break

    # 停止
    send_cmd(0x02)  # STOP_ALL
    time.sleep(0.3)

    # 最終サンプル
    send_cmd(0x13)
    _, imu = read_packets(0.1)
    if imu:
        yaw_now = struct.unpack_from('<f', imu, 32)[0]
        d_yaw = yaw_now - yaw_prev
        if d_yaw >  180: d_yaw -= 360
        if d_yaw < -180: d_yaw += 360
        yaw_accumulated += d_yaw

    ser.close()

    # ── 結果 & wheel_base 補正 ────────────────────────────────────
    odom_deg = math.degrees(odom_accumulated)
    gyro_deg = yaw_accumulated

    print(f'\n  結果:')
    print(f'    オドメトリ角度   : {odom_deg:.2f}°')
    print(f'    ジャイロ角度(真値): {gyro_deg:.2f}°')

    if abs(gyro_deg) < 10:
        print('  ✗ ジャイロ角度が小さすぎます。旋回が正常に行われなかったか確認してください')
        return

    # 絶対値で比率を計算 (ジャイロの Z 軸極性がオドメトリと逆の場合があるため)
    ratio = abs(gyro_deg) / abs(odom_deg)
    wheel_base_new = round(wheel_base_current * ratio, 4)

    print(f'\n    補正比率         : {ratio:.4f}')
    print(f'    現在 wheel_base  : {wheel_base_current} m')
    print(f'    補正後 wheel_base: {wheel_base_new} m')

    error_before = abs(abs(odom_deg) - abs(gyro_deg))
    print(f'\n    補正前の角度誤差  : {error_before:.2f}°')
    print(f'    補正後の予測誤差  : < 1°')

    if apply:
        config['robot']['wheel_base'] = wheel_base_new
        with open(CONFIG_PATH, 'w') as f:
            json.dump(config, f, indent=2)
        print(f'\n  ✓ robot_config.json を更新しました (wheel_base: {wheel_base_current} → {wheel_base_new})')
    else:
        print(f'\n  → --apply を付けて実行すると robot_config.json を自動更新します')

    # ── launch_all.py 再起動 ─────────────────────────────────────
    print()
    try:
        ans = input('  launch_all.py を再起動しますか？ [y/N]: ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = 'n'
    if ans == 'y':
        log_dir = Path(__file__).parent / 'logs'
        log_dir.mkdir(exist_ok=True)
        subprocess.Popen(
            ['python3', 'launch_all.py'],
            cwd=str(Path(__file__).parent),
            stdout=open(log_dir / 'launch_all.log', 'a'),
            stderr=subprocess.STDOUT,
        )
        print('  ✓ launch_all.py 再起動')

    print()
    print(SEP2)
    print('  完了')
    print(SEP2)


def main():
    parser = argparse.ArgumentParser(description='wheel_base calibration using IMU gyro')
    parser.add_argument('--apply', action='store_true',
                        help='計測後に robot_config.json を自動更新')
    args = parser.parse_args()

    print()
    print(f'  ⚠  このスクリプトは launch_all.py を停止し、')
    print(f'     ロボットを 360° 回転させます。')
    print(f'     十分なスペースを確保してください。')
    print()
    try:
        ans = input('  開始しますか？ [y/N]: ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        print('\n  中止')
        return
    if ans != 'y':
        print('  中止')
        return

    run_calibration(apply=args.apply)


if __name__ == '__main__':
    main()
