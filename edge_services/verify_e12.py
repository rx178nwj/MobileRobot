#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E12
=====================================
E12: 複数クライアント同時接続

検証内容:
  [1] odom サービスに 2 クライアント同時接続 → 両方がデータを受信できるか
  [2] odom クライアント A を切断 → クライアント B への配信が継続するか
  [3] camera サービスに 2 クライアント同時接続 → 両方がフレームを受信できるか
  [4] camera クライアント A を切断 → クライアント B への配信が継続するか

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e12.py
"""

import asyncio
import json
import websockets

from _verify_common import ODOM_URI, SEP, SEP2, check_services

PASS = '✓ PASS'
FAIL = '✗ FAIL'

CAMERA_URI   = 'ws://localhost:8003'
RECV_TIMEOUT = 3.0   # seconds
CONT_MSGS    = 5     # 切断後に継続確認するメッセージ数


async def main():
    print()
    print(SEP2)
    print('  [E12] 複数クライアント同時接続検証')
    print(SEP2)
    print()
    if not await check_services():
        return

    results = []

    # ════════════════════════════════════════════════════════
    # Odom サービス
    # ════════════════════════════════════════════════════════
    print(f'  {SEP}')
    print('  [1] odom: 2 クライアント同時接続・同時受信')
    print(f'  {SEP}')

    async with websockets.connect(ODOM_URI) as ws_a, \
               websockets.connect(ODOM_URI) as ws_b:

        # 両クライアントで初回メッセージを受信
        try:
            msg_a = json.loads(await asyncio.wait_for(ws_a.recv(), timeout=RECV_TIMEOUT))
            msg_b = json.loads(await asyncio.wait_for(ws_b.recv(), timeout=RECV_TIMEOUT))
            ok_a = msg_a.get('type') == 'odom'
            ok_b = msg_b.get('type') == 'odom'
            print(f'  クライアントA: type={msg_a.get("type")}  pose={msg_a["pose"]}')
            print(f'  クライアントB: type={msg_b.get("type")}  pose={msg_b["pose"]}')
            if ok_a and ok_b:
                print(f'  {PASS}: 両クライアントが odom を受信')
                results.append(True)
            else:
                print(f'  {FAIL}: 一方または両方が odom を受信できませんでした')
                results.append(False)
        except asyncio.TimeoutError:
            print(f'  {FAIL}: タイムアウト ({RECV_TIMEOUT}s)')
            results.append(False)

        # ── [2] A を切断 → B への配信継続 ────────────────────
        print()
        print(f'  {SEP}')
        print('  [2] odom: クライアントA 切断 → クライアントB への配信継続')
        print(f'  {SEP}')

        await ws_a.close()
        print('  クライアントA 切断完了')

        recv_count = 0
        try:
            for _ in range(CONT_MSGS):
                msg = json.loads(await asyncio.wait_for(ws_b.recv(), timeout=RECV_TIMEOUT))
                if msg.get('type') == 'odom':
                    recv_count += 1
            print(f'  クライアントB: 切断後 {recv_count}/{CONT_MSGS} メッセージ受信')
            if recv_count >= CONT_MSGS:
                print(f'  {PASS}: A 切断後も B への配信が継続しています')
                results.append(True)
            else:
                print(f'  {FAIL}: 受信メッセージ不足 ({recv_count}/{CONT_MSGS})')
                results.append(False)
        except asyncio.TimeoutError:
            print(f'  {FAIL}: タイムアウト — B への配信が停止した可能性があります')
            results.append(False)
        except websockets.exceptions.ConnectionClosed as e:
            print(f'  {FAIL}: クライアントB の接続が切断されました: {e}')
            results.append(False)

    # ════════════════════════════════════════════════════════
    # Camera サービス
    # ════════════════════════════════════════════════════════
    print()
    print(f'  {SEP}')
    print('  [3] camera: 2 クライアント同時接続・同時受信')
    print(f'  {SEP}')

    async with websockets.connect(CAMERA_URI, max_size=10*1024*1024) as cam_a, \
               websockets.connect(CAMERA_URI, max_size=10*1024*1024) as cam_b:

        # camera_info を読み捨て
        try:
            info_a_raw = await asyncio.wait_for(cam_a.recv(), timeout=RECV_TIMEOUT)
            info_b_raw = await asyncio.wait_for(cam_b.recv(), timeout=RECV_TIMEOUT)
            info_a = json.loads(info_a_raw if isinstance(info_a_raw, str) else info_a_raw.decode())
            info_b = json.loads(info_b_raw if isinstance(info_b_raw, str) else info_b_raw.decode())
            print(f'  カメラA camera_info: {info_a}')
            print(f'  カメラB camera_info: {info_b}')
        except Exception as e:
            print(f'  {FAIL}: camera_info 受信失敗: {e}')
            results.append(False)
            results.append(False)
            return

        # 両クライアントでフレームを受信
        try:
            frame_a = await asyncio.wait_for(cam_a.recv(), timeout=RECV_TIMEOUT)
            frame_b = await asyncio.wait_for(cam_b.recv(), timeout=RECV_TIMEOUT)
            ok_a = isinstance(frame_a, (bytes, bytearray)) and len(frame_a) > 8
            ok_b = isinstance(frame_b, (bytes, bytearray)) and len(frame_b) > 8
            print(f'  カメラA フレームサイズ: {len(frame_a) if ok_a else "NG"} bytes')
            print(f'  カメラB フレームサイズ: {len(frame_b) if ok_b else "NG"} bytes')
            if ok_a and ok_b:
                print(f'  {PASS}: 両クライアントがカメラフレームを受信')
                results.append(True)
            else:
                print(f'  {FAIL}: 一方または両方がフレームを受信できませんでした')
                results.append(False)
        except asyncio.TimeoutError:
            print(f'  {FAIL}: タイムアウト ({RECV_TIMEOUT}s)')
            results.append(False)

        # ── [4] カメラ A を切断 → B への配信継続 ─────────────
        print()
        print(f'  {SEP}')
        print('  [4] camera: クライアントA 切断 → クライアントB への配信継続')
        print(f'  {SEP}')

        await cam_a.close()
        print('  カメラクライアントA 切断完了')

        recv_count = 0
        try:
            for _ in range(CONT_MSGS):
                frame = await asyncio.wait_for(cam_b.recv(), timeout=RECV_TIMEOUT)
                if isinstance(frame, (bytes, bytearray)) and len(frame) > 8:
                    recv_count += 1
            print(f'  カメラB: 切断後 {recv_count}/{CONT_MSGS} フレーム受信')
            if recv_count >= CONT_MSGS:
                print(f'  {PASS}: A 切断後も B へのフレーム配信が継続しています')
                results.append(True)
            else:
                print(f'  {FAIL}: 受信フレーム不足 ({recv_count}/{CONT_MSGS})')
                results.append(False)
        except asyncio.TimeoutError:
            print(f'  {FAIL}: タイムアウト — B への配信が停止した可能性があります')
            results.append(False)
        except websockets.exceptions.ConnectionClosed as e:
            print(f'  {FAIL}: カメラクライアントB の接続が切断されました: {e}')
            results.append(False)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E12 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E12 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
