#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E2
=====================================
E2: WebSocket 全ポート接続確認

検証内容:
  [1] ポート 8001 (motor)  接続確立
  [2] ポート 8002 (odom)   接続確立 + 初回 odom メッセージ確認
  [3] ポート 8003 (camera) 接続確立 + camera_info メッセージ確認

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e2.py
"""

import asyncio
import json
import struct
import websockets

MOTOR_URI  = 'ws://localhost:8001'
ODOM_URI   = 'ws://localhost:8002'
CAMERA_URI = 'ws://localhost:8003'
TIMEOUT    = 3.0

SEP  = '─' * 58
SEP2 = '═' * 58
PASS = '✓ PASS'
FAIL = '✗ FAIL'


async def main():
    print()
    print(SEP2)
    print('  [E2] WebSocket 全ポート接続確認')
    print(SEP2)
    print()

    results = []

    # ── [1] ポート 8001 motor ────────────────────────────────────
    print(f'  {SEP}')
    print('  [1] ポート 8001 (motor_service) 接続確認')
    print(f'  {SEP}')
    try:
        async with websockets.connect(MOTOR_URI, open_timeout=TIMEOUT) as ws:
            print(f'  {PASS}: ws://localhost:8001 接続確立')
            # motor はコマンド受信型なので送信してACKを確認
            await ws.send(json.dumps({'type': 'stop'}))
            resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=TIMEOUT))
            if resp.get('type') == 'ack':
                print(f'    stop コマンド ACK: {resp}')
                results.append(True)
            else:
                print(f'  {FAIL}: 予期しないレスポンス: {resp}')
                results.append(False)
    except Exception as e:
        print(f'  {FAIL}: 接続失敗 — {e}')
        results.append(False)

    # ── [2] ポート 8002 odom ─────────────────────────────────────
    print()
    print(f'  {SEP}')
    print('  [2] ポート 8002 (odom_service) 接続確認')
    print(f'  {SEP}')
    try:
        async with websockets.connect(ODOM_URI, open_timeout=TIMEOUT) as ws:
            print(f'  {PASS}: ws://localhost:8002 接続確立')
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=TIMEOUT))
            if msg.get('type') == 'odom':
                pose = msg['pose']
                twist = msg['twist']
                encs  = msg['encoders']
                print(f'    初回 odom メッセージ受信:')
                print(f'      pose  : x={pose["x"]:.4f}  y={pose["y"]:.4f}  θ={pose["theta"]:.4f} rad')
                print(f'      twist : linear={twist["linear"]:.4f} m/s  angular={twist["angular"]:.4f} rad/s')
                print(f'      encoders: {encs}')
                results.append(True)
            else:
                print(f'  {FAIL}: 期待するメッセージ type=odom ではありません: {msg.get("type")}')
                results.append(False)
    except Exception as e:
        print(f'  {FAIL}: 接続失敗 — {e}')
        results.append(False)

    # ── [3] ポート 8003 camera ───────────────────────────────────
    print()
    print(f'  {SEP}')
    print('  [3] ポート 8003 (camera_service) 接続確認')
    print(f'  {SEP}')
    try:
        async with websockets.connect(CAMERA_URI, open_timeout=TIMEOUT) as ws:
            print(f'  {PASS}: ws://localhost:8003 接続確立')
            raw = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT)

            # camera_info は JSON 文字列、フレームはバイナリ
            if isinstance(raw, str):
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    # camera_service sends str(dict) — use ast.literal_eval
                    import ast
                    msg = ast.literal_eval(raw)
                if msg.get('type') == 'camera_info':
                    print(f'    camera_info 受信:')
                    print(f'      解像度 : {msg["width"]}×{msg["height"]}')
                    print(f'      FPS    : {msg.get("fps", "N/A")}')
                    results.append(True)
                else:
                    print(f'  {FAIL}: 予期しない JSON: {msg}')
                    results.append(False)
            elif isinstance(raw, bytes):
                # フレームが先に来た場合はヘッダーを解析
                if len(raw) >= 8:
                    ts, w, h = struct.unpack_from('<fHH', raw, 0)
                    jpeg_size = len(raw) - 8
                    print(f'    バイナリフレーム受信 (camera_info より先):')
                    print(f'      解像度   : {w}×{h}')
                    print(f'      timestamp: {ts:.3f}')
                    print(f'      JPEG サイズ: {jpeg_size} bytes')
                    results.append(True)
                else:
                    print(f'  {FAIL}: バイナリデータが短すぎます ({len(raw)} bytes)')
                    results.append(False)
            else:
                print(f'  {FAIL}: 予期しない型: {type(raw)}')
                results.append(False)
    except Exception as e:
        print(f'  {FAIL}: 接続失敗 — {e}')
        results.append(False)

    # ── サマリー ─────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    print()
    print(SEP2)
    if passed == total:
        print(f'  E2 結果: {PASS}  ({passed}/{total} 項目)')
    else:
        print(f'  E2 結果: {FAIL}  ({passed}/{total} 項目通過)')
    print(SEP2)
    print()


if __name__ == '__main__':
    asyncio.run(main())
