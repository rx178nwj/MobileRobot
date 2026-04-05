#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E7
=====================================
E7: オドメトリリセット

検証内容:
  [1] リセット前に pose が非ゼロであること (走行して蓄積)
  [2] {"type": "reset"} 送信後に x/y/theta が 0 になること
  [3] エンコーダーカウントもリセットされること

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e7.py
"""

import asyncio
import json
import math
import websockets

from _verify_common import (
    MOTOR_URI, ODOM_URI, SEP, SEP2,
    send_cmd_vel, stop, get_odom, check_services,
)

PASS = '✓ PASS'
FAIL = '✗ FAIL'


async def main():
    print()
    print(SEP2)
    print('  [E7] オドメトリリセット検証')
    print(SEP2)
    print()
    if not await check_services():
        return

    results = []

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # ── [1] 走行して pose を蓄積 ─────────────────────────────
        print(f'  {SEP}')
        print('  [1] 走行して pose を非ゼロに')
        print(f'  {SEP}')

        # まず現在値を確認
        odom_init = await get_odom(odom_ws)
        print(f'  走行前: x={odom_init["pose"]["x"]:.4f}  y={odom_init["pose"]["y"]:.4f}  θ={math.degrees(odom_init["pose"]["theta"]):.2f}°  enc={odom_init["encoders"]}')

        # 0.5秒走行
        for _ in range(10):
            await send_cmd_vel(motor_ws, 0.15, 0.0)
            await asyncio.sleep(0.05)
        await stop(motor_ws)
        await asyncio.sleep(0.3)

        odom_before = await get_odom(odom_ws)
        x_before   = odom_before['pose']['x']
        y_before   = odom_before['pose']['y']
        th_before  = odom_before['pose']['theta']
        enc_before = odom_before['encoders']
        print(f'  走行後: x={x_before:.4f}  y={y_before:.4f}  θ={math.degrees(th_before):.2f}°  enc={enc_before}')

        dist = math.sqrt(x_before**2 + y_before**2)
        if dist > 0.01 or any(e != 0 for e in enc_before):
            print(f'  {PASS}: pose が非ゼロ (距離={dist:.4f}m)')
            results.append(True)
        else:
            print(f'  ⚠  走行しても pose が変化しませんでした (距離={dist:.4f}m)')
            results.append(False)

        # ── [2] リセット送信 → x/y/theta が 0 ──────────────────
        print()
        print(f'  {SEP}')
        print('  [2] {"type": "reset"} 送信 → pose リセット確認')
        print(f'  {SEP}')

        await odom_ws.send(json.dumps({'type': 'reset'}))
        ack = json.loads(await asyncio.wait_for(odom_ws.recv(), timeout=2.0))
        print(f'  ACK: {ack}')

        await asyncio.sleep(0.1)
        odom_after = await get_odom(odom_ws)
        x_after  = odom_after['pose']['x']
        y_after  = odom_after['pose']['y']
        th_after = odom_after['pose']['theta']
        print(f'  リセット後: x={x_after:.4f}  y={y_after:.4f}  θ={math.degrees(th_after):.2f}°')

        THRESH = 0.005  # 5mm / 0.3°
        if abs(x_after) < THRESH and abs(y_after) < THRESH and abs(th_after) < math.radians(0.3):
            print(f'  {PASS}: x/y/theta がゼロにリセットされました')
            results.append(True)
        else:
            print(f'  {FAIL}: リセット後も非ゼロです (x={x_after:.4f}, y={y_after:.4f}, θ={math.degrees(th_after):.2f}°)')
            results.append(False)

        # ── [3] エンコーダーカウントのリセット確認 ──────────────
        print()
        print(f'  {SEP}')
        print('  [3] エンコーダーカウント リセット確認')
        print(f'  {SEP}')

        enc_after = odom_after['encoders']
        print(f'  リセット後 enc: {enc_after}')

        if all(abs(e) < 10 for e in enc_after):
            print(f'  {PASS}: エンコーダーカウントがゼロにリセットされました')
            results.append(True)
        else:
            print(f'  {FAIL}: エンコーダーが非ゼロです {enc_after}')
            results.append(False)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E7 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E7 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
