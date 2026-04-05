#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E5
=====================================
E5: 直進精度 — 1m 直進後の誤差 <5cm

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e5.py
  python3 verify_e5.py --yes   # 確認プロンプトをスキップ
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


async def test_e5():
    print(f'\n{SEP2}')
    print('  [E5] 直進精度検証')
    print(f'  仕様: 1m 直進後の誤差 <5cm')
    print(SEP2)
    print('  ⚠  ロボットが前進します。1m 以上のスペースを確保してください。')
    if not ask('開始しますか？'):
        print('  スキップ')
        return

    LINEAR_VEL  = 0.15   # m/s (低速で精度向上)
    TARGET_DIST = 1.0    # m
    STOP_AT     = 0.98   # 98% 到達でコマンド停止、残りはコースト (~0.02m)
    MAX_TIME    = 20.0   # 安全タイムアウト

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # オドメトリリセット (theta=0、向き蓄積をクリア)
        await odom_ws.send(json.dumps({'type': 'reset'}))
        await asyncio.wait_for(odom_ws.recv(), timeout=2.0)
        await asyncio.sleep(0.1)

        odom0 = await get_odom(odom_ws)
        x0 = odom0['pose']['x']
        y0 = odom0['pose']['y']
        print(f'\n  開始位置: x={x0:.4f} m, y={y0:.4f} m')
        print(f'  速度: {LINEAR_VEL} m/s  目標: {TARGET_DIST} m (距離ベース停止)')
        print(f'\n  走行中...')

        # 距離ベース停止
        t_start = time.time()
        while (time.time() - t_start) < MAX_TIME:
            await send_cmd_vel(motor_ws, LINEAR_VEL, 0.0)
            odom = await get_odom(odom_ws)
            traveled_now = math.sqrt(
                (odom['pose']['x'] - x0)**2 + (odom['pose']['y'] - y0)**2)
            if traveled_now >= TARGET_DIST * STOP_AT:
                break
            await asyncio.sleep(0.05)

        await stop(motor_ws)
        await asyncio.sleep(0.3)  # コースト待ち

        odom1 = await get_odom(odom_ws)
        x1 = odom1['pose']['x']
        y1 = odom1['pose']['y']

        traveled = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        error_m  = abs(traveled - TARGET_DIST)
        lateral  = abs(y1 - y0)

        print(f'\n  結果 (オドメトリ計測):')
        print(f'    目標距離    : {TARGET_DIST:.3f} m')
        print(f'    計測距離    : {traveled:.4f} m')
        print(f'    縦誤差      : {error_m*100:.1f} cm')
        print(f'    横ずれ      : {lateral*100:.1f} cm')

        PASS_CM = 5.0
        if error_m * 100 < PASS_CM:
            print(f'\n  ✓ PASS (オドメトリ): 誤差 {error_m*100:.1f} cm < {PASS_CM} cm')
        else:
            print(f'\n  △ オドメトリ誤差 {error_m*100:.1f} cm ≥ {PASS_CM} cm')

        print(f'\n  📏 物理計測: 実際の移動距離をメジャーで確認してください')
        try:
            actual = float(input('  実測値を入力 (m, スキップは Enter): ').strip() or '0')
            if actual > 0:
                phys_err = abs(actual - TARGET_DIST) * 100
                print(f'    実測誤差: {phys_err:.1f} cm', end='  ')
                print('✓ PASS' if phys_err < PASS_CM else '✗ FAIL')
        except ValueError:
            pass


async def main():
    print()
    print(SEP2)
    print('  Edge Service 動作検証  E5')
    print(SEP2)
    print()
    if not await check_services():
        return
    await test_e5()
    print()
    print(SEP2)
    print('  検証完了')
    print(SEP2)
    print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Edge service verification E5')
    parser.add_argument('--yes', action='store_true', help='確認プロンプトをスキップして自動実行')
    args = parser.parse_args()

    if args.yes:
        import builtins
        builtins.input = lambda prompt='': (print(prompt + 'y  (--yes)'), 'y')[1]

    asyncio.run(main())
