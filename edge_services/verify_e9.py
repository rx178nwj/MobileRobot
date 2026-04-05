#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E9
=====================================
E9: カメラ画像受信

検証内容:
  [1] ws://localhost:8003 に接続して camera_info メッセージを受信できるか
  [2] バイナリフレームを受信できるか
  [3] ヘッダー (8B: timestamp 4B float LE + width 2B LE + height 2B LE) が正しく解析できるか
  [4] JPEG デコードが成功するか (PIL / cv2 を使わず JPEG SOI/EOI マーカーで確認)

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e9.py
"""

import asyncio
import json
import struct
import time
import websockets

from _verify_common import SEP, SEP2

PASS = '✓ PASS'
FAIL = '✗ FAIL'

CAMERA_URI    = 'ws://localhost:8003'
RECV_TIMEOUT  = 5.0   # seconds — カメラフレームの初回受信タイムアウト
HEADER_SIZE   = 8     # bytes
EXPECTED_W    = 640
EXPECTED_H    = 480

JPEG_SOI = b'\xff\xd8'  # Start Of Image
JPEG_EOI = b'\xff\xd9'  # End Of Image


async def main():
    print()
    print(SEP2)
    print('  [E9] カメラ画像受信検証')
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

    async with websockets.connect(CAMERA_URI) as ws:

        # ── [1] camera_info 受信 ────────────────────────────────
        print(f'  {SEP}')
        print('  [1] camera_info メッセージ受信')
        print(f'  {SEP}')

        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=3.0)
            if isinstance(raw, bytes):
                raw = raw.decode('utf-8', errors='replace')
            info = json.loads(raw)
            print(f'  受信: {info}')
            if info.get('type') == 'camera_info':
                w = info.get('width')
                h = info.get('height')
                fps = info.get('fps')
                fmt = info.get('format')
                print(f'  解像度: {w}x{h}  fps: {fps}  format: {fmt}')
                if w == EXPECTED_W and h == EXPECTED_H:
                    print(f'  {PASS}: camera_info 受信 ({w}x{h})')
                    results.append(True)
                else:
                    print(f'  {FAIL}: 解像度不一致 (期待 {EXPECTED_W}x{EXPECTED_H}、実際 {w}x{h})')
                    results.append(False)
            else:
                print(f'  {FAIL}: type が camera_info ではありません: {info.get("type")}')
                results.append(False)
        except Exception as e:
            print(f'  {FAIL}: camera_info 受信エラー: {e}')
            results.append(False)

        # ── [2] バイナリフレーム受信 ────────────────────────────
        print()
        print(f'  {SEP}')
        print(f'  [2] バイナリフレーム受信 (タイムアウト: {RECV_TIMEOUT}s)')
        print(f'  {SEP}')

        frame_data = None
        try:
            frame_data = await asyncio.wait_for(ws.recv(), timeout=RECV_TIMEOUT)
            if isinstance(frame_data, (bytes, bytearray)):
                print(f'  受信サイズ: {len(frame_data)} bytes')
                if len(frame_data) > HEADER_SIZE:
                    print(f'  {PASS}: バイナリフレーム受信 ({len(frame_data)} bytes)')
                    results.append(True)
                else:
                    print(f'  {FAIL}: フレームサイズが小さすぎます ({len(frame_data)} bytes)')
                    results.append(False)
            else:
                print(f'  {FAIL}: バイナリではなくテキストが届きました: {str(frame_data)[:80]}')
                results.append(False)
                frame_data = None
        except asyncio.TimeoutError:
            print(f'  {FAIL}: {RECV_TIMEOUT}秒以内にフレームが届きませんでした')
            results.append(False)
        except Exception as e:
            print(f'  {FAIL}: フレーム受信エラー: {e}')
            results.append(False)

        # ── [3] ヘッダー解析 ────────────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [3] ヘッダー解析 (8 bytes: timestamp/width/height)')
        print(f'  {SEP}')

        if frame_data and len(frame_data) >= HEADER_SIZE:
            try:
                ts, w, h = struct.unpack_from('<fHH', frame_data, 0)
                now = time.time()
                age_sec = now - ts
                print(f'  timestamp : {ts:.3f}  (age {age_sec*1000:.0f} ms)')
                print(f'  width     : {w}')
                print(f'  height    : {h}')

                ok_wh = (w == EXPECTED_W and h == EXPECTED_H)
                # float32 の精度は ~1.775e9 で約 ±128s なので 300s 以内を合格とする
                ok_ts = (ts > 1e9 and abs(age_sec) < 300.0)

                if ok_wh and ok_ts:
                    print(f'  {PASS}: ヘッダー正常 ({w}x{h}, age={age_sec*1000:.0f}ms)')
                    print(f'         ※ timestamp は float32 (4B) のため精度 ±128s')
                    results.append(True)
                else:
                    if not ok_wh:
                        print(f'  {FAIL}: 解像度不一致 (期待 {EXPECTED_W}x{EXPECTED_H}、実際 {w}x{h})')
                    if not ok_ts:
                        print(f'  {FAIL}: timestamp 異常 (age={age_sec:.1f}s)')
                    results.append(False)
            except struct.error as e:
                print(f'  {FAIL}: ヘッダー解析失敗: {e}')
                results.append(False)
        else:
            print(f'  スキップ (フレーム未受信)')
            results.append(False)

        # ── [4] JPEG マーカー確認 ────────────────────────────────
        print()
        print(f'  {SEP}')
        print('  [4] JPEG データ確認 (SOI/EOI マーカー)')
        print(f'  {SEP}')

        if frame_data and len(frame_data) > HEADER_SIZE:
            jpeg_bytes = frame_data[HEADER_SIZE:]
            has_soi = jpeg_bytes[:2] == JPEG_SOI
            has_eoi = jpeg_bytes[-2:] == JPEG_EOI
            print(f'  JPEG サイズ : {len(jpeg_bytes)} bytes')
            print(f'  SOI (FFD8)  : {"OK" if has_soi else "NG"} — 先頭 {jpeg_bytes[:2].hex()}')
            print(f'  EOI (FFD9)  : {"OK" if has_eoi else "NG"} — 末尾 {jpeg_bytes[-2:].hex()}')

            if has_soi and has_eoi:
                print(f'  {PASS}: JPEG マーカー正常')
                results.append(True)
            else:
                print(f'  {FAIL}: JPEG マーカー異常')
                results.append(False)
        else:
            print(f'  スキップ (フレーム未受信)')
            results.append(False)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E9 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E9 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
