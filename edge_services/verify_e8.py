#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E8
=====================================
E8: オドメトリ発行レート (50Hz)

検証内容:
  [1] 1秒間に受信したメッセージ数が 45〜55 の範囲か
  [2] メッセージ間隔の平均・最大・最小が仕様内か

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e8.py
"""

import asyncio
import json
import time
import websockets

from _verify_common import ODOM_URI, SEP, SEP2, check_services

PASS = '✓ PASS'
FAIL = '✗ FAIL'

SAMPLE_SEC   = 3.0   # 計測時間
RATE_MIN     = 45    # Hz
RATE_MAX     = 55    # Hz
INTERVAL_MAX = 30.0  # ms — 最大許容間隔


async def main():
    print()
    print(SEP2)
    print('  [E8] オドメトリ発行レート検証 (目標: 50 Hz)')
    print(SEP2)
    print()
    if not await check_services():
        return

    results = []

    async with websockets.connect(ODOM_URI) as ws:
        await ws.recv()  # initial msg

        # ── 計測 ─────────────────────────────────────────────────
        print(f'  {SEP}')
        print(f'  [1] {SAMPLE_SEC}秒間のメッセージ受信レート計測')
        print(f'  {SEP}')
        print(f'  計測中...')

        timestamps = []
        t_end = time.time() + SAMPLE_SEC
        while time.time() < t_end:
            try:
                await asyncio.wait_for(ws.recv(), timeout=0.1)
                timestamps.append(time.time())
            except asyncio.TimeoutError:
                pass

        count = len(timestamps)
        rate  = count / SAMPLE_SEC

        print(f'\n  受信メッセージ数 : {count} msgs / {SAMPLE_SEC:.0f}s')
        print(f'  平均レート       : {rate:.1f} Hz')

        if RATE_MIN <= rate <= RATE_MAX:
            print(f'  {PASS}: レート {rate:.1f} Hz ∈ [{RATE_MIN}, {RATE_MAX}] Hz')
            results.append(True)
        else:
            print(f'  {FAIL}: レート {rate:.1f} Hz ∉ [{RATE_MIN}, {RATE_MAX}] Hz')
            results.append(False)

        # ── [2] 間隔分析 ────────────────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [2] メッセージ間隔分析')
        print(f'  {SEP}')

        if len(timestamps) >= 2:
            intervals = [(timestamps[i+1] - timestamps[i]) * 1000
                         for i in range(len(timestamps)-1)]
            avg_ms = sum(intervals) / len(intervals)
            min_ms = min(intervals)
            max_ms = max(intervals)

            print(f'  平均間隔 : {avg_ms:.1f} ms  (目標: 20.0 ms)')
            print(f'  最小間隔 : {min_ms:.1f} ms')
            print(f'  最大間隔 : {max_ms:.1f} ms  (許容: < {INTERVAL_MAX:.0f} ms)')

            if max_ms < INTERVAL_MAX:
                print(f'  {PASS}: 最大間隔 {max_ms:.1f} ms < {INTERVAL_MAX:.0f} ms')
                results.append(True)
            else:
                print(f'  {FAIL}: 最大間隔 {max_ms:.1f} ms ≥ {INTERVAL_MAX:.0f} ms (レート抜けあり)')
                results.append(False)
        else:
            print(f'  {FAIL}: サンプル不足')
            results.append(False)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E8 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E8 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
