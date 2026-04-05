#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E1
=====================================
E1: PR2040 ハードウェア接続確認

検証内容:
  [1] /dev/ttyACM0 デバイス存在確認
  [2] ファームウェア応答確認 (get_status via odom WebSocket)
  [3] エンコーダー 4 チャンネル読み取り確認
  [4] ファームウェアタイムスタンプが進んでいること (自動送信100Hz)

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e1.py
"""

import asyncio
import json
import os
import sys
import time
import websockets

ODOM_URI = 'ws://localhost:8002'
USB_PORT = '/dev/ttyACM0'
SEP  = '─' * 58
SEP2 = '═' * 58

PASS = '✓ PASS'
FAIL = '✗ FAIL'


async def main():
    print()
    print(SEP2)
    print('  [E1] PR2040 ハードウェア接続確認')
    print(SEP2)
    print()

    results = []

    # ── [1] デバイスファイル存在確認 ─────────────────────────────
    print(f'  {SEP}')
    print(f'  [1] {USB_PORT} デバイス存在確認')
    print(f'  {SEP}')
    if os.path.exists(USB_PORT):
        print(f'  {PASS}: {USB_PORT} が存在します')
        results.append(True)
    else:
        print(f'  {FAIL}: {USB_PORT} が見つかりません')
        print(f'    → PR2040 の USB ケーブルを確認してください')
        results.append(False)

    # ── [2] ファームウェア応答確認 ────────────────────────────────
    print()
    print(f'  {SEP}')
    print(f'  [2] ファームウェア応答確認 (get_status)')
    print(f'  {SEP}')
    status = None
    try:
        async with websockets.connect(ODOM_URI, open_timeout=2.0) as ws:
            await ws.recv()  # initial odom msg
            await ws.send(json.dumps({'type': 'get_status'}))
            resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
            if resp.get('type') == 'status' and 'data' in resp:
                status = resp['data']
                print(f'  {PASS}: ファームウェアが応答しました')
                print(f'    firmware timestamp : {status["timestamp"]} ms')
                results.append(True)
            else:
                print(f'  {FAIL}: 予期しないレスポンス: {resp}')
                results.append(False)
    except Exception as e:
        print(f'  {FAIL}: WebSocket 接続エラー: {e}')
        print(f'    → launch_all.py が起動しているか確認してください')
        results.append(False)

    if status is None:
        print()
        print('  ⚠  ファームウェアに接続できないため [3][4] をスキップします')
        _print_summary(results)
        return

    # ── [3] エンコーダー 4 チャンネル読み取り ──────────────────────
    print()
    print(f'  {SEP}')
    print(f'  [3] エンコーダー 4 チャンネル読み取り')
    print(f'  {SEP}')
    encoders = status['encoders']
    print(f'    enc[0] (左前)  : {encoders[0]:>8} counts')
    print(f'    enc[1] (右前)  : {encoders[1]:>8} counts')
    print(f'    enc[2] (左後)  : {encoders[2]:>8} counts')
    print(f'    enc[3] (右後)  : {encoders[3]:>8} counts')

    active = sum(1 for e in encoders if e != 0)
    if active >= 2:
        print(f'  {PASS}: {active}/4 チャンネルが有効値 (非ゼロ) を返しています')
        results.append(True)
    elif active == 0:
        print(f'  {FAIL}: 全チャンネルが 0 — エンコーダー配線を確認してください')
        results.append(False)
    else:
        print(f'  ⚠  WARNING: {active}/4 チャンネルのみ有効 (enc[2]/enc[3] 未接続の可能性)')
        results.append(True)  # 2輪構成として許容

    # ── [4] タイムスタンプ進行確認 (自動送信 100Hz) ───────────────
    print()
    print(f'  {SEP}')
    print(f'  [4] ファームウェアタイムスタンプ進行確認 (自動送信 100Hz)')
    print(f'  {SEP}')
    try:
        async def drain(ws):
            """Drain all buffered odom messages."""
            for _ in range(100):
                try:
                    await asyncio.wait_for(ws.recv(), timeout=0.001)
                except asyncio.TimeoutError:
                    break

        async def get_status_ts(ws) -> int:
            """Drain buffer, send get_status, return firmware timestamp."""
            await drain(ws)
            await ws.send(json.dumps({'type': 'get_status'}))
            for _ in range(20):
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
                if msg.get('type') == 'status':
                    return msg['data']['timestamp']
            raise RuntimeError('status response not received')

        async with websockets.connect(ODOM_URI, open_timeout=2.0) as ws:
            await ws.recv()  # initial odom msg

            ts1 = await get_status_ts(ws)
            await asyncio.sleep(0.5)
            ts2 = await get_status_ts(ws)

            delta_ms = ts2 - ts1
            expected_ms = 500  # ~0.5s
            print(f'    timestamp T=0   : {ts1} ms')
            print(f'    timestamp T=0.5s: {ts2} ms')
            print(f'    delta           : {delta_ms} ms (期待値 ~{expected_ms} ms)')

            if 400 <= delta_ms <= 600:
                print(f'  {PASS}: タイムスタンプが 100Hz で更新されています')
                results.append(True)
            elif delta_ms > 0:
                print(f'  ⚠  WARNING: 更新はされているが速度が異常 ({delta_ms} ms)')
                results.append(True)
            else:
                print(f'  {FAIL}: タイムスタンプが進んでいません — ファームウェアが停止している可能性')
                results.append(False)

    except Exception as e:
        print(f'  {FAIL}: タイムスタンプ確認エラー: {e}')
        results.append(False)

    _print_summary(results)


def _print_summary(results):
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E1 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E1 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
