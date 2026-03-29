#!/usr/bin/env python3
"""
Duty Sweep Test — 実デッドゾーン計測
======================================
motor_service.py が起動中のまま WebSocket 経由で動作します。

仕組み:
  cmd_vel の linear 値を逆算して duty を間接的に指定
  duty ≈ FF_GAIN * target_cps + min_duty_kickstart

  ただし PI+FF の kickstart ロジックをバイパスするため、
  このスクリプトは launch_all.py を一時停止して直接 PR2040 を操作します。

Usage:
  cd ~/MobileRobot/edge_services
  python3 duty_sweep.py          # 自動計測
  python3 duty_sweep.py --apply  # 計測後に motor_service.py を自動更新
"""

import sys
import os
import time
import math
import signal
import subprocess
import argparse

sys.path.insert(0, os.path.dirname(__file__))

WHEEL_RADIUS = 0.033
ENCODER_CPR  = 827.2
SEP  = "─" * 58
SEP2 = "═" * 58

def velocity_to_cps(v: float) -> float:
    return (v / (2.0 * math.pi * WHEEL_RADIUS)) * ENCODER_CPR

def cps_to_mps(cps: float) -> float:
    return cps / ENCODER_CPR * (2.0 * math.pi * WHEEL_RADIUS)


def sweep(apply_fix: bool):
    from hardware.pr2040_usb_driver import PR2040USBDriver

    print(SEP2)
    print("  Duty Sweep — デッドゾーン実測")
    print(SEP2)
    print("  launch_all.py を停止して直接計測します")
    print()

    # ── 1. Stop launch_all.py ──
    result = subprocess.run(
        ["pgrep", "-f", "launch_all.py"], capture_output=True, text=True)
    pids = result.stdout.strip().split()
    if pids:
        print(f"  launch_all.py を停止 (PID: {' '.join(pids)}) ...")
        for pid in pids:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
        print("  ✓ 停止")
    else:
        print("  launch_all.py は未起動")

    # ── 2. Connect to PR2040 ──
    print("  PR2040 接続中...")
    time.sleep(0.5)
    driver = PR2040USBDriver(port='/dev/ttyACM0')
    if not driver.connected:
        print("  ✗ 接続失敗")
        return
    print("  ✓ 接続OK")
    time.sleep(0.5)

    # ── 3. Duty sweep ──
    duties = [150, 200, 250, 300, 320, 340, 360, 380,
              400, 420, 440, 460, 480, 500, 550, 600, 700, 800]
    results = []
    dead_zone_end = None

    print()
    print(f"  {'duty':>5}  {'L vel[cps]':>11}  {'R vel[cps]':>11}  "
          f"{'avg[cps]':>9}  {'avg[m/s]':>9}  {'状態'}")
    print(f"  {SEP}")

    for duty in duties:
        driver.set_wheel_duties((duty, -duty, duty, -duty))
        time.sleep(0.6)

        samples = []
        for _ in range(5):
            s = driver.get_status()
            samples.append(s['velocities'])
            time.sleep(0.04)

        driver.stop_all()
        time.sleep(0.3)

        avg_L = sum(s[0]  for s in samples) / len(samples)
        avg_R = sum(-s[1] for s in samples) / len(samples)
        avg   = (avg_L + avg_R) / 2.0
        mps   = cps_to_mps(abs(avg))
        moving = abs(avg) > 50

        if moving and dead_zone_end is None:
            dead_zone_end = duty

        mark = "✓ 動作" if moving else "✗ 停止"
        results.append((duty, avg_L, avg_R, avg, mps, moving))
        print(f"  {duty:>5}  {avg_L:>11.0f}  {avg_R:>11.0f}  "
              f"{avg:>9.0f}  {mps:>9.3f}  {mark}")

    driver.close()

    # ── 4. Analysis ──
    print()
    print(SEP2)
    print("  解析結果")
    print(SEP2)

    if dead_zone_end is None:
        print("  ✗ duty=800 でも動作しません")
        print("    → モーター配線・電源・PR2040 ファームウェアを確認してください")
        return

    # Speed at dead zone threshold
    row = next(r for r in results if r[0] == dead_zone_end)
    speed_at_min = abs(row[3])
    mps_at_min   = row[4]

    # Speed at duty=420 (current MIN_DUTY)
    row_420 = next((r for r in results if r[0] == 420), None)
    if row_420:
        speed_420 = abs(row_420[3])
        mps_420   = row_420[4]
    else:
        speed_420 = None

    target_cps = velocity_to_cps(0.5)

    print(f"  実デッドゾーン終端    : duty = {dead_zone_end}")
    print(f"  デッドゾーン出口速度  : {speed_at_min:.0f} cps = {mps_at_min:.3f} m/s")
    print(f"  0.5 m/s の目標        : {target_cps:.0f} cps")
    if speed_420 is not None:
        print(f"  duty=420 での実速度   : {speed_420:.0f} cps = {mps_420:.3f} m/s")
    print()

    if speed_at_min > target_cps * 1.1:
        print(f"  ⚠  0.5 m/s ({target_cps:.0f} cps) はデッドゾーン内です")
        print(f"     デッドゾーン出口速度 ({speed_at_min:.0f} cps = {mps_at_min:.3f} m/s) の方が速い")
        print()
        print(f"  ► 推奨対応: nav2 の最小速度を {mps_at_min:.2f} m/s 以上に設定")
        print(f"     OR  nav2_params.yaml の min_vel_x を {mps_at_min:.2f} に変更")
        suggested_min_vel = round(mps_at_min + 0.05, 2)
        print(f"     推奨 min_vel_x = {suggested_min_vel}")
    else:
        print(f"  ✓ 0.5 m/s はデッドゾーン外です")
        print(f"  ► MIN_DUTY を {dead_zone_end + 20} に設定することを推奨")

    # Build calibration table
    print()
    print("  速度キャリブレーション表:")
    print(f"  {'duty':>5}  {'m/s':>7}  {'cps':>7}")
    for row in results:
        if row[5]:  # moving
            print(f"  {row[0]:>5}  {row[4]:>7.3f}  {abs(row[3]):>7.0f}")

    # ── 5. Auto-apply fix ──
    if apply_fix:
        _apply_fix(dead_zone_end, mps_at_min, speed_at_min, target_cps)

    # ── 6. Restart launch_all.py ──
    print()
    ans = input("  launch_all.py を再起動しますか？ [y/N]: ").strip().lower()
    if ans == 'y':
        log_dir = os.path.join(os.path.dirname(__file__), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        subprocess.Popen(
            ["python3", "launch_all.py"],
            cwd=os.path.dirname(__file__),
            stdout=open(f"{log_dir}/launch_all.log", "a"),
            stderr=subprocess.STDOUT,
        )
        print("  ✓ launch_all.py 再起動")


def _apply_fix(dead_zone_end, mps_at_min, speed_at_min, target_cps):
    """Update motor_service.py with calibrated values."""
    import re

    service_path = os.path.join(os.path.dirname(__file__), 'motor_service.py')
    with open(service_path) as f:
        src = f.read()

    if speed_at_min > target_cps * 1.1:
        # Dead zone includes 0.5 m/s — update min_duty to calibrated value
        new_min = dead_zone_end + 20
        src_new = re.sub(
            r"'min_duty':\s*\d+",
            f"'min_duty':     {new_min}",
            src
        )
        print(f"\n  motor_service.py: min_duty → {new_min} に更新")
    else:
        new_min = dead_zone_end + 20
        src_new = re.sub(
            r"'min_duty':\s*\d+",
            f"'min_duty':     {new_min}",
            src
        )
        print(f"\n  motor_service.py: min_duty → {new_min} に更新")

    with open(service_path, 'w') as f:
        f.write(src_new)
    print(f"  ✓ 保存完了: {service_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--apply', action='store_true',
                        help='計測後に motor_service.py を自動更新')
    args = parser.parse_args()

    print()
    print(f"  ⚠  このスクリプトは launch_all.py を停止し、")
    print(f"     ロボットを実際に動かします。")
    print(f"     平坦な場所に置き、十分なスペースを確保してください。")
    print()
    try:
        ans = input("  計測を開始しますか？ [y/N]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\n  中止")
        return

    if ans != 'y':
        print("  中止")
        return

    sweep(apply_fix=args.apply)

    print()
    print(SEP2)
    print("  完了")
    print(SEP2)


if __name__ == '__main__':
    main()
