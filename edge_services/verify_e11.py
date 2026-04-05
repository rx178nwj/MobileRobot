#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E11
=====================================
E11: 速度制限クランプ

検証内容:
  [1] linear=1.0 m/s (制限 0.5 m/s 超) 送信 → ACK linear ≤ 0.5 m/s
  [2] angular=3.0 rad/s (制限 2.0 rad/s 超) 送信 → ACK angular ≤ 2.0 rad/s
  [3] 負方向: linear=-1.0 → ACK linear ≥ -0.5 m/s
  [4] 制限内指令はそのまま通過: linear=0.3, angular=1.0

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e11.py
"""

import asyncio
import json
import websockets

from _verify_common import MOTOR_URI, SEP, SEP2, check_services

PASS = '✓ PASS'
FAIL = '✗ FAIL'

MAX_LINEAR  = 0.5   # m/s
MAX_ANGULAR = 2.0   # rad/s


async def send_and_ack(ws, linear: float, angular: float) -> dict:
    await ws.send(json.dumps({'type': 'cmd_vel', 'linear': linear, 'angular': angular}))
    raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
    return json.loads(raw)


async def main():
    print()
    print(SEP2)
    print('  [E11] 速度制限クランプ検証')
    print(SEP2)
    print()
    if not await check_services():
        return

    results = []

    async with websockets.connect(MOTOR_URI) as ws:

        # ── [1] linear 正方向クランプ ────────────────────────────
        print(f'  {SEP}')
        print('  [1] linear=1.0 m/s 送信 → クランプ ≤ 0.5 m/s')
        print(f'  {SEP}')

        ack = await send_and_ack(ws, linear=1.0, angular=0.0)
        ack_linear = ack.get('linear', float('nan'))
        print(f'  送信: linear=1.0  ACK linear={ack_linear}')
        if ack_linear <= MAX_LINEAR + 1e-9:
            print(f'  {PASS}: ACK linear={ack_linear:.3f} ≤ {MAX_LINEAR}')
            results.append(True)
        else:
            print(f'  {FAIL}: ACK linear={ack_linear:.3f} > {MAX_LINEAR} (クランプされていません)')
            results.append(False)

        await ws.send(json.dumps({'type': 'stop'}))
        await asyncio.wait_for(ws.recv(), timeout=2.0)

        # ── [2] angular クランプ ─────────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [2] angular=3.0 rad/s 送信 → クランプ ≤ 2.0 rad/s')
        print(f'  {SEP}')

        ack = await send_and_ack(ws, linear=0.0, angular=3.0)
        ack_angular = ack.get('angular', float('nan'))
        print(f'  送信: angular=3.0  ACK angular={ack_angular}')
        if ack_angular <= MAX_ANGULAR + 1e-9:
            print(f'  {PASS}: ACK angular={ack_angular:.3f} ≤ {MAX_ANGULAR}')
            results.append(True)
        else:
            print(f'  {FAIL}: ACK angular={ack_angular:.3f} > {MAX_ANGULAR} (クランプされていません)')
            results.append(False)

        await ws.send(json.dumps({'type': 'stop'}))
        await asyncio.wait_for(ws.recv(), timeout=2.0)

        # ── [3] linear 負方向クランプ ────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [3] linear=-1.0 m/s 送信 → クランプ ≥ -0.5 m/s')
        print(f'  {SEP}')

        ack = await send_and_ack(ws, linear=-1.0, angular=0.0)
        ack_linear = ack.get('linear', float('nan'))
        print(f'  送信: linear=-1.0  ACK linear={ack_linear}')
        if ack_linear >= -MAX_LINEAR - 1e-9:
            print(f'  {PASS}: ACK linear={ack_linear:.3f} ≥ {-MAX_LINEAR}')
            results.append(True)
        else:
            print(f'  {FAIL}: ACK linear={ack_linear:.3f} < {-MAX_LINEAR} (クランプされていません)')
            results.append(False)

        await ws.send(json.dumps({'type': 'stop'}))
        await asyncio.wait_for(ws.recv(), timeout=2.0)

        # ── [4] 制限内はそのまま通過 ─────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [4] linear=0.3, angular=1.0 (制限内) → 値が変化しないこと')
        print(f'  {SEP}')

        ack = await send_and_ack(ws, linear=0.3, angular=1.0)
        ack_linear  = ack.get('linear',  float('nan'))
        ack_angular = ack.get('angular', float('nan'))
        print(f'  送信: linear=0.3 angular=1.0  ACK linear={ack_linear} angular={ack_angular}')

        ok_l = abs(ack_linear  - 0.3) < 1e-6
        ok_a = abs(ack_angular - 1.0) < 1e-6
        if ok_l and ok_a:
            print(f'  {PASS}: 制限内の値がそのまま通過しました')
            results.append(True)
        else:
            print(f'  {FAIL}: 値が変わっています (linear={ack_linear:.4f}, angular={ack_angular:.4f})')
            results.append(False)

        await ws.send(json.dumps({'type': 'stop'}))
        await asyncio.wait_for(ws.recv(), timeout=2.0)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E11 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E11 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
