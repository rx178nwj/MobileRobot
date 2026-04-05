#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E10
=====================================
E10: カメラフレームレート (30fps)

検証内容:
  [1] 1秒間の受信フレーム数が 25〜35 の範囲か
  [2] フレームサイズ (width/height) がヘッダー値と設定値 (640×480) と一致するか

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e10.py
"""

import asyncio
import json
import struct
import time
import websockets

from _verify_common import SEP, SEP2

PASS = '✓ PASS'
FAIL = '✗ FAIL'

CAMERA_URI   = 'ws://localhost:8003'
SAMPLE_SEC   = 3.0    # 計測時間
RATE_MIN     = 25     # fps
RATE_MAX     = 35     # fps
HEADER_SIZE  = 8      # bytes
EXPECTED_W   = 640
EXPECTED_H   = 480


async def main():
    print()
    print(SEP2)
    print('  [E10] カメラフレームレート検証 (目標: 30 fps)')
    print(SEP2)
    print()

    # ── サービス確認 ────────────────────────────────────────────
    print(f'  camera_service  {CAMERA_URI}')
    print()
    try:
        async with websockets.connect(CAMERA_URI, open_timeout=2.0):
            print('  ✓ camera_service 接続OK')
    except Exception as e:
        print(f'  ✗ camera_service 接続失敗: {e}')
        print('    → launch_all.py が起動しているか確認してください')
        print()
        return

    print()
    results = []

    async with websockets.connect(CAMERA_URI, max_size=10*1024*1024) as ws:

        # camera_info を読み捨て
        raw = await asyncio.wait_for(ws.recv(), timeout=3.0)
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', errors='replace')
        info = json.loads(raw)
        print(f'  camera_info: {info}')
        print()

        # ── [1] フレームレート計測 ────────────────────────────────
        print(f'  {SEP}')
        print(f'  [1] {SAMPLE_SEC:.0f}秒間のフレーム受信レート計測')
        print(f'  {SEP}')
        print(f'  計測中...')

        timestamps = []
        t_end = time.time() + SAMPLE_SEC
        while time.time() < t_end:
            try:
                await asyncio.wait_for(ws.recv(), timeout=0.2)
                timestamps.append(time.time())
            except asyncio.TimeoutError:
                pass

        count = len(timestamps)
        rate  = count / SAMPLE_SEC

        print(f'\n  受信フレーム数 : {count} frames / {SAMPLE_SEC:.0f}s')
        print(f'  平均レート     : {rate:.1f} fps')

        if RATE_MIN <= rate <= RATE_MAX:
            print(f'  {PASS}: レート {rate:.1f} fps ∈ [{RATE_MIN}, {RATE_MAX}] fps')
            results.append(True)
        else:
            print(f'  {FAIL}: レート {rate:.1f} fps ∉ [{RATE_MIN}, {RATE_MAX}] fps')
            results.append(False)

        # フレーム間隔も表示
        if len(timestamps) >= 2:
            intervals = [(timestamps[i+1] - timestamps[i]) * 1000
                         for i in range(len(timestamps)-1)]
            avg_ms = sum(intervals) / len(intervals)
            min_ms = min(intervals)
            max_ms = max(intervals)
            print(f'  平均間隔: {avg_ms:.1f} ms  最小: {min_ms:.1f} ms  最大: {max_ms:.1f} ms')

        # ── [2] フレームサイズ確認 ────────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [2] フレームサイズ (ヘッダー値と設定値の一致確認)')
        print(f'  {SEP}')

        # 新たにフレームを1枚受信してヘッダーを確認
        try:
            frame_data = await asyncio.wait_for(ws.recv(), timeout=5.0)
        except asyncio.TimeoutError:
            print(f'  {FAIL}: フレーム受信タイムアウト')
            results.append(False)
            frame_data = None

        if frame_data and len(frame_data) >= HEADER_SIZE:
            _, w, h = struct.unpack_from('<fHH', frame_data, 0)
            jpeg_size = len(frame_data) - HEADER_SIZE
            print(f'  ヘッダー width  : {w}  (期待: {EXPECTED_W})')
            print(f'  ヘッダー height : {h}  (期待: {EXPECTED_H})')
            print(f'  JPEG サイズ     : {jpeg_size} bytes')

            if w == EXPECTED_W and h == EXPECTED_H:
                print(f'  {PASS}: フレームサイズ一致 ({w}x{h})')
                results.append(True)
            else:
                print(f'  {FAIL}: フレームサイズ不一致 (ヘッダー {w}x{h} ≠ 期待 {EXPECTED_W}x{EXPECTED_H})')
                results.append(False)
        elif frame_data:
            print(f'  {FAIL}: フレームが短すぎます ({len(frame_data)} bytes)')
            results.append(False)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E10 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E10 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
