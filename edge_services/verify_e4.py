#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E4
=====================================
E4: クライアント切断時の自動停止

検証内容:
  [1] cmd_vel を送信しながら WebSocket を強制切断
  [2] 切断後 odom で速度が 0 に落ちることを確認

motor_service.py の handle_client finally: stop_motors() の動作確認。
E3 (タイムアウト停止) と補完関係 — こちらは切断イベントによる即時停止。

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e4.py
"""

import asyncio
import json
import math
import time
import websockets

MOTOR_URI = 'ws://localhost:8001'
ODOM_URI  = 'ws://localhost:8002'
TIMEOUT   = 3.0

SEP  = '─' * 58
SEP2 = '═' * 58
PASS = '✓ PASS'
FAIL = '✗ FAIL'

WARMUP_SEC  = 2.0   # モーターが定常速度に達するまで
LINEAR_VEL  = 0.3   # m/s
VEL_THRESH  = 0.05  # m/s — これ以下を「停止」とみなす
WAIT_AFTER  = 1.0   # 切断後の確認待ち


async def get_latest_odom(odom_ws) -> dict:
    msg = await asyncio.wait_for(odom_ws.recv(), timeout=TIMEOUT)
    for _ in range(500):
        try:
            msg = await asyncio.wait_for(odom_ws.recv(), timeout=0.001)
        except asyncio.TimeoutError:
            break
    return json.loads(msg)


async def main():
    print()
    print(SEP2)
    print('  [E4] クライアント切断時の自動停止')
    print(SEP2)
    print()
    print('  仕様: cmd_vel 送信中に WebSocket を切断 → モーター即時停止')
    print('  ⚠  モーターが短時間動きます。安全な場所に置いてください。')
    print()

    results = []

    # ── [1] 走行中に切断 ─────────────────────────────────────────
    print(f'  {SEP}')
    print('  [1] cmd_vel 送信中に WebSocket 切断')
    print(f'  {SEP}')

    try:
        # odom は切断後も監視するため別コンテキストで保持
        odom_ws = await websockets.connect(ODOM_URI, open_timeout=TIMEOUT)

        # ウォームアップ: モーターを定常速度まで加速
        print(f'\n  モーター起動中 ({WARMUP_SEC}s)...')
        async with websockets.connect(MOTOR_URI, open_timeout=TIMEOUT) as motor_ws:
            t0 = time.time()
            while time.time() - t0 < WARMUP_SEC:
                await motor_ws.send(json.dumps(
                    {'type': 'cmd_vel', 'linear': LINEAR_VEL, 'angular': 0.0}))
                await asyncio.wait_for(motor_ws.recv(), timeout=TIMEOUT)
                await asyncio.sleep(0.1)

            odom = await get_latest_odom(odom_ws)
            vel_before = abs(odom['twist']['linear'])
            print(f'  切断前の速度: {vel_before:.3f} m/s')

            # ─ ここで motor_ws のコンテキストを抜けて切断 ─
            print('  WebSocket 切断...')

        # async with を抜けると切断完了
        t_disconnect = time.time()

        if vel_before < VEL_THRESH:
            print(f'  ⚠  WARNING: 切断前の速度が低すぎます ({vel_before:.3f} m/s)')
            print(f'     デッドゾーンを抜けていない可能性があります')

        results.append(vel_before > VEL_THRESH)

    except Exception as e:
        print(f'  {FAIL}: 走行フェーズでエラー — {e}')
        results.append(False)
        return

    # ── [2] 切断後の速度確認 ─────────────────────────────────────
    print()
    print(f'  {SEP}')
    print(f'  [2] 切断後 {WAIT_AFTER}s の速度確認')
    print(f'  {SEP}')

    try:
        await asyncio.sleep(WAIT_AFTER)
        odom = await get_latest_odom(odom_ws)
        vel_after = abs(odom['twist']['linear'])
        elapsed = time.time() - t_disconnect

        print(f'\n  結果:')
        print(f'    切断前の速度  : {vel_before:.3f} m/s')
        print(f'    {elapsed:.1f}s 後の速度  : {vel_after:.3f} m/s')

        if vel_after < VEL_THRESH:
            print(f'\n  {PASS}: 切断後にモーターが自動停止 ({vel_after:.3f} m/s < {VEL_THRESH})')
            results.append(True)
        else:
            print(f'\n  {FAIL}: 停止していません ({vel_after:.3f} m/s ≥ {VEL_THRESH})')
            print(f'    → motor_service.py の handle_client finally を確認してください')
            results.append(False)

    except Exception as e:
        print(f'  {FAIL}: 速度確認でエラー — {e}')
        results.append(False)
    finally:
        await odom_ws.close()

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E4 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E4 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
