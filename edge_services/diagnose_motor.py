#!/usr/bin/env python3
"""
Motor Diagnostic Script
========================
Diagnoses why /cmd_vel=0.5 doesn't move the robot.

Tests performed:
  [1] PR2040 USB connection & status
  [2] Duty sweep  — find actual dead zone (min duty to move)
  [3] cmd_vel=0.5 simulation  — what duty does motor_service compute?
  [4] PI+FF live test  — run the full control loop for 3 seconds

Usage:
  cd ~/MobileRobot/edge_services
  python3 diagnose_motor.py
"""

import sys
import os
import time
import math
import struct
import threading

sys.path.insert(0, os.path.dirname(__file__))

from hardware.pr2040_usb_driver import PR2040USBDriver

# ── Robot constants (must match motor_service.py) ──────────────────────────
WHEEL_RADIUS  = 0.033    # m
ENCODER_CPR   = 827.2    # counts/rev
WHEEL_BASE    = 0.16     # m

# ── PI + FF gains (must match motor_service.py) ────────────────────────────
FF_GAIN       = 800.0 / 5771.0   # ≈ 0.139
KP            = 0.05
KI            = 0.08
MIN_DUTY      = 420
MAX_DUTY      = 1000
MAX_INTEGRAL  = 400.0

SEP  = "─" * 60
SEP2 = "═" * 60


def velocity_to_cps(v_mps: float) -> float:
    return (v_mps / (2.0 * math.pi * WHEEL_RADIUS)) * ENCODER_CPR


def ask_continue(prompt="続行しますか？ [y/N]: ") -> bool:
    try:
        return input(prompt).strip().lower() == 'y'
    except (EOFError, KeyboardInterrupt):
        return False


# ── Test 1: PR2040 connection ──────────────────────────────────────────────

def test_connection(driver: PR2040USBDriver):
    print(f"\n{SEP2}")
    print("  [Test 1] PR2040 USB 接続確認")
    print(SEP2)

    if not driver.connected:
        print("  ✗ PR2040 未接続  →  /dev/ttyACM0 を確認してください")
        return False

    status = driver.get_status()
    enc = status['encoders']
    vel = status['velocities']
    print(f"  ✓ PR2040 接続OK")
    print(f"  エンコーダ  : {enc}")
    print(f"  速度 [cps]  : {vel}")
    print(f"  タイムスタンプ: {status['timestamp']} ms")
    return True


# ── Test 2: Duty sweep ─────────────────────────────────────────────────────

def test_duty_sweep(driver: PR2040USBDriver):
    print(f"\n{SEP2}")
    print("  [Test 2] Duty スイープ  —  デッドゾーン検出")
    print(SEP2)
    print("  ⚠  ロボットが動きます。安全な場所に置いてください。")
    if not ask_continue("  開始しますか？ [y/N]: "):
        print("  スキップ")
        return None

    duties   = [200, 300, 350, 380, 400, 420, 450, 500, 600, 700, 800]
    results  = []
    dead_zone_end = None

    print(f"\n  {'Duty':>6}  {'Left vel [cps]':>16}  {'Right vel [cps]':>16}  {'動作'}")
    print(f"  {SEP}")

    for duty in duties:
        # Apply duty for 0.5 s
        driver.set_wheel_duties((duty, -duty, duty, -duty))
        time.sleep(0.5)

        # Read velocity (average over 3 samples)
        samples = []
        for _ in range(3):
            s = driver.get_status()
            samples.append(s['velocities'])
            time.sleep(0.05)
        driver.stop_all()
        time.sleep(0.3)

        avg_left  = sum(s[0] for s in samples) / 3
        avg_right = -sum(s[1] for s in samples) / 3  # mirrored
        moving    = abs(avg_left) > 30 or abs(avg_right) > 30
        mark      = "✓ 動作" if moving else "✗ 停止"

        if moving and dead_zone_end is None:
            dead_zone_end = duty

        results.append((duty, avg_left, avg_right, moving))
        print(f"  {duty:>6}  {avg_left:>16.0f}  {avg_right:>16.0f}  {mark}")

    print()
    if dead_zone_end:
        cps_at_min = (results[[r[0] for r in results].index(dead_zone_end)][1] +
                      results[[r[0] for r in results].index(dead_zone_end)][2]) / 2
        mps_at_min = cps_at_min / ENCODER_CPR * (2 * math.pi * WHEEL_RADIUS)
        print(f"  ► デッドゾーン終端  : duty = {dead_zone_end}")
        print(f"  ► デッドゾーン出口速度: {cps_at_min:.0f} cps  ≈  {mps_at_min:.3f} m/s")
        print(f"  ► 現在の MIN_DUTY   : {MIN_DUTY}")
        if MIN_DUTY < dead_zone_end:
            print(f"  ✗ MIN_DUTY({MIN_DUTY}) < 実デッドゾーン({dead_zone_end})")
            print(f"    → min_duty を {dead_zone_end + 20} 以上に設定してください")
        else:
            print(f"  ✓ MIN_DUTY({MIN_DUTY}) >= 実デッドゾーン({dead_zone_end})  →  設定OK")
    else:
        print("  ✗ duty=800 でも動作しませんでした  →  配線・電源を確認してください")

    return dead_zone_end


# ── Test 3: cmd_vel=0.5 duty calculation ──────────────────────────────────

def test_cmd_vel_calculation():
    print(f"\n{SEP2}")
    print("  [Test 3] cmd_vel=0.5 の Duty 計算確認")
    print(SEP2)

    target_mps = 0.5
    target_cps = velocity_to_cps(target_mps)

    # FF term
    ff = FF_GAIN * target_cps

    # PI starts at 0 (no error compensation yet)
    raw = ff + 0 + 0   # ff + P(0) + I(0)

    # Dead zone kickstart
    if abs(raw) < MIN_DUTY:
        after_kickstart = MIN_DUTY
        kickstart_applied = True
    else:
        after_kickstart = raw
        kickstart_applied = False

    print(f"  目標速度          : {target_mps} m/s")
    print(f"  目標速度 [cps]    : {target_cps:.1f} cps")
    print(f"  Feedforward (FF)  : {FF_GAIN:.4f} × {target_cps:.1f} = {ff:.1f}")
    print(f"  初期 PI 出力      : 0  (起動直後)")
    print(f"  Raw 出力          : {raw:.1f}")
    print(f"  MIN_DUTY          : {MIN_DUTY}")
    print()

    if kickstart_applied:
        print(f"  ⚠  FF出力({ff:.0f}) < MIN_DUTY({MIN_DUTY})")
        print(f"     → kickstart 適用: duty = {after_kickstart}")
        print(f"     → モーター起動後 PI が目標速度に収束させます")
    else:
        print(f"  ✓ FF出力({ff:.0f}) >= MIN_DUTY({MIN_DUTY})")
        print(f"     → 直接 duty = {after_kickstart:.0f} で起動")

    print()
    # Check if MIN_DUTY produces speed > target (potential overshoot)
    # Approximate: speed at MIN_DUTY ≈ MIN_DUTY / FF_GAIN
    approx_cps_at_min = MIN_DUTY / FF_GAIN
    approx_mps_at_min = approx_cps_at_min / ENCODER_CPR * (2 * math.pi * WHEEL_RADIUS)
    print(f"  MIN_DUTYでの推定速度: {approx_cps_at_min:.0f} cps ≈ {approx_mps_at_min:.3f} m/s")
    if approx_mps_at_min > target_mps * 1.5:
        print(f"  ⚠  kickstart速度が目標の{approx_mps_at_min/target_mps:.1f}倍 → PI積分が逆方向に働く")
        print(f"     → kp/ki を上げるか、min_duty を下げると改善します")
    else:
        print(f"  ✓ kickstart速度は目標の{approx_mps_at_min/target_mps:.1f}倍  →  PIで収束可能")


# ── Test 4: PI+FF live control loop ────────────────────────────────────────

def test_piFF_live(driver: PR2040USBDriver, dead_zone: int = None):
    print(f"\n{SEP2}")
    print("  [Test 4] PI + Feedforward ライブ制御テスト (3秒間)")
    print(SEP2)
    print("  ⚠  ロボットが前進します。安全な場所に置いてください。")
    if not ask_continue("  開始しますか？ [y/N]: "):
        print("  スキップ")
        return

    target_cps    = velocity_to_cps(0.5)
    min_duty_use  = max(MIN_DUTY, (dead_zone or MIN_DUTY))
    integral      = 0.0
    prev_time     = time.time()
    dt_loop       = 0.02   # 50 Hz

    print(f"\n  目標: 0.5 m/s = {target_cps:.0f} cps   min_duty={min_duty_use}")
    print(f"\n  {'時刻[s]':>8}  {'目標[cps]':>10}  {'実速[cps]':>10}  {'duty':>6}  {'誤差[cps]':>10}")
    print(f"  {SEP}")

    start = time.time()
    try:
        while (time.time() - start) < 3.0:
            now = time.time()
            dt  = now - prev_time
            prev_time = now

            # Read actual velocity
            status      = driver.get_status()
            vel         = status['velocities']
            actual_left = float(vel[0])
            actual_right = -float(vel[1])
            actual_avg  = (actual_left + actual_right) / 2.0

            error     = target_cps - actual_avg
            integral += error * dt
            integral  = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, integral))

            raw    = FF_GAIN * target_cps + KP * error + KI * integral
            sign   = 1 if raw >= 0 else -1
            if abs(raw) < min_duty_use:
                raw = sign * min_duty_use
            duty   = int(max(-MAX_DUTY, min(MAX_DUTY, raw)))

            driver.set_wheel_duties((duty, -duty, duty, -duty))

            elapsed = now - start
            if int(elapsed / 0.1) != int((elapsed - dt) / 0.1):  # print every 100ms
                print(f"  {elapsed:>8.2f}  {target_cps:>10.0f}  {actual_avg:>10.0f}  "
                      f"{duty:>6}  {error:>10.0f}")

            time.sleep(dt_loop)

    except KeyboardInterrupt:
        print("\n  中断")
    finally:
        driver.stop_all()
        print(f"\n  ✓ テスト完了  →  モーター停止")

    # Summary
    print()
    status = driver.get_status()
    final_vel = status['velocities']
    print(f"  最終エンコーダ速度: left={final_vel[0]} right={-final_vel[1]} cps")


# ── Test 5: WebSocket motor service ────────────────────────────────────────

def test_websocket():
    print(f"\n{SEP2}")
    print("  [Test 5] WebSocket motor_service 接続テスト")
    print(SEP2)

    try:
        import asyncio
        import websockets
        import json

        async def _test():
            uri = "ws://localhost:8001"
            print(f"  接続先: {uri}")
            try:
                async with websockets.connect(uri, open_timeout=2.0) as ws:
                    print("  ✓ WebSocket 接続成功")

                    # Send cmd_vel=0.5
                    cmd = {"type": "cmd_vel", "linear": 0.5, "angular": 0.0}
                    await ws.send(json.dumps(cmd))
                    print(f"  送信: {cmd}")

                    resp = await asyncio.wait_for(ws.recv(), timeout=1.0)
                    print(f"  応答: {resp}")
                    print("  ✓ motor_service は cmd_vel を受信しています")

                    # Stop
                    await ws.send(json.dumps({"type": "stop"}))
                    await asyncio.wait_for(ws.recv(), timeout=1.0)
            except Exception as e:
                print(f"  ✗ 接続失敗: {e}")
                print("    → motor_service が起動しているか確認してください")
                print("      cd ~/MobileRobot/edge_services && python3 launch_all.py")

        asyncio.run(_test())

    except ImportError:
        print("  websockets モジュールが見つかりません: pip3 install websockets")


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    print(SEP2)
    print("  Motor Diagnostic  —  /cmd_vel=0.5 追従不可の原因調査")
    print(SEP2)
    print(f"  設定値:")
    print(f"    WHEEL_RADIUS = {WHEEL_RADIUS} m    ENCODER_CPR = {ENCODER_CPR}")
    print(f"    FF_GAIN = {FF_GAIN:.4f}   KP = {KP}   KI = {KI}")
    print(f"    MIN_DUTY = {MIN_DUTY}   MAX_DUTY = {MAX_DUTY}")
    print(f"    0.5 m/s = {velocity_to_cps(0.5):.0f} cps")

    # Connect to PR2040
    print(f"\n  PR2040 へ接続中 (/dev/ttyACM0)...")
    driver = PR2040USBDriver(port='/dev/ttyACM0')

    if not test_connection(driver):
        print("\n  PR2040 に接続できないため診断を中止します")
        return

    # Run tests
    test_cmd_vel_calculation()
    dead_zone = test_duty_sweep(driver)
    test_piFF_live(driver, dead_zone)
    test_websocket()

    driver.close()
    print(f"\n{SEP2}")
    print("  診断完了")
    print(SEP2)
    print()
    print("  ■ 次のアクション:")
    print("  1. デッドゾーンが MIN_DUTY を超えていた場合:")
    print("     motor_service.py の pid.min_duty を実測値+20 に変更して再起動")
    print()
    print("  2. WebSocket テストが失敗した場合:")
    print("     cd ~/MobileRobot/edge_services && python3 launch_all.py")
    print()
    print("  3. ライブテストで duty=420 でも動かない場合:")
    print("     duty=600, 700 で Test 2 を再実施して実デッドゾーンを確認")
    print()


if __name__ == '__main__':
    main()
