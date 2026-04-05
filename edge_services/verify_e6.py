#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E6
=====================================
E6: 旋回精度 — 360° 旋回後の誤差 <5°

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e6.py
  python3 verify_e6.py --yes   # 確認プロンプトをスキップ
"""

import argparse
import asyncio
import json
import math
import time
import websockets

from _verify_common import (
    MOTOR_URI, ODOM_URI, SEP, SEP2,
    ask, send_cmd_vel, stop, get_odom, check_services,
)


async def test_e6():
    print(f'\n{SEP2}')
    print('  [E6] 旋回精度検証')
    print(f'  仕様: 360° 旋回後の誤差 <5°')
    print(SEP2)
    print('  ⚠  ロボットが 360° 回転します。十分なスペースを確保してください。')
    if not ask('開始しますか？'):
        print('  スキップ')
        return

    # 最小信頼角速度 ~1.84 rad/s (dead zone = 440 duty = 587 cps)
    # 2.0 rad/s (638 cps) / ki=2.0 → デッドゾーン突破 ~0.30s
    ANGULAR_VEL = 2.0           # rad/s
    TARGET_RAD  = 2.0 * math.pi # 360°
    STOP_AT     = 0.990         # wheel_base=0.181m 再調整後: 354.9°+5.1°補正
    MAX_TIME    = 15.0          # 安全タイムアウト

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # オドメトリリセット (前テストの蓄積をクリア)
        await odom_ws.send(json.dumps({'type': 'reset'}))
        await asyncio.wait_for(odom_ws.recv(), timeout=2.0)
        await asyncio.sleep(0.1)

        odom0 = await get_odom(odom_ws)
        theta0 = odom0['pose']['theta']
        print(f'\n  開始角度: {math.degrees(theta0):.2f}°')
        print(f'  角速度: {ANGULAR_VEL} rad/s  目標: 360° (角度ベース停止)')
        print(f'\n  旋回中...')

        # 角度ベース停止
        theta_prev  = theta0
        accumulated = 0.0
        t_start     = time.time()

        while (time.time() - t_start) < MAX_TIME:
            await send_cmd_vel(motor_ws, 0.0, ANGULAR_VEL)
            odom = await get_odom(odom_ws)
            theta_now = odom['pose']['theta']

            d_theta = theta_now - theta_prev
            if d_theta >  math.pi: d_theta -= 2 * math.pi
            if d_theta < -math.pi: d_theta += 2 * math.pi
            accumulated += d_theta
            theta_prev = theta_now

            if abs(accumulated) >= TARGET_RAD * STOP_AT:
                break
            await asyncio.sleep(0.05)

        await stop(motor_ws)
        await asyncio.sleep(0.3)

        odom1 = await get_odom(odom_ws)

        error_rad = abs(accumulated - TARGET_RAD)
        error_deg = math.degrees(error_rad)

        print(f'\n  結果 (オドメトリ計測):')
        print(f'    目標角度    : 360.00°')
        print(f'    計測角度    : {math.degrees(accumulated):.2f}°')
        print(f'    角度誤差    : {error_deg:.2f}°')

        PASS_DEG = 5.0
        if error_deg < PASS_DEG:
            print(f'\n  ✓ PASS (オドメトリ): 誤差 {error_deg:.2f}° < {PASS_DEG}°')
        else:
            print(f'\n  △ オドメトリ誤差 {error_deg:.2f}° ≥ {PASS_DEG}°')

        print(f'\n  📐 物理確認: ロボットの向きが出発方向と同じか目視確認してください')
        try:
            actual_deg = float(input('  実測角度を入力 (°, スキップは Enter): ').strip() or '0')
            if actual_deg > 0:
                phys_err = abs(actual_deg - 360.0)
                print(f'    実測誤差: {phys_err:.1f}°', end='  ')
                print('✓ PASS' if phys_err < PASS_DEG else '✗ FAIL')
        except ValueError:
            pass


async def main():
    print()
    print(SEP2)
    print('  Edge Service 動作検証  E6')
    print(SEP2)
    print()
    if not await check_services():
        return
    await test_e6()
    print()
    print(SEP2)
    print('  検証完了')
    print(SEP2)
    print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Edge service verification E6')
    parser.add_argument('--yes', action='store_true', help='確認プロンプトをスキップして自動実行')
    args = parser.parse_args()

    if args.yes:
        import builtins
        builtins.input = lambda prompt='': (print(prompt + 'y  (--yes)'), 'y')[1]

    asyncio.run(main())
