"""
共通ユーティリティ — Edge Service 検証スクリプト群
"""

import asyncio
import json
import websockets

MOTOR_URI = 'ws://localhost:8001'
ODOM_URI  = 'ws://localhost:8002'

SEP  = '─' * 58
SEP2 = '═' * 58


def ask(prompt: str) -> bool:
    try:
        return input(f'  {prompt} [y/N]: ').strip().lower() == 'y'
    except (EOFError, KeyboardInterrupt):
        return False


async def send_cmd_vel(ws, linear: float, angular: float):
    await ws.send(json.dumps({'type': 'cmd_vel', 'linear': linear, 'angular': angular}))
    await asyncio.wait_for(ws.recv(), timeout=1.0)


async def stop(ws):
    await ws.send(json.dumps({'type': 'stop'}))
    await asyncio.wait_for(ws.recv(), timeout=1.0)


async def get_odom(odom_ws) -> dict:
    """最新の odom メッセージを返す (バッファの古いメッセージをドレイン)。

    パブリッシャーは 50 Hz (20ms/msg)。1ms タイムアウトでドレインすることで
    キューが空になった時点で停止し、次の live メッセージを取り込まない。
    最大 500 回でループを終了する。
    """
    msg = await asyncio.wait_for(odom_ws.recv(), timeout=2.0)
    for _ in range(500):
        try:
            msg = await asyncio.wait_for(odom_ws.recv(), timeout=0.001)
        except asyncio.TimeoutError:
            break
    return json.loads(msg)


async def check_services() -> bool:
    """motor / odom サービスへの接続確認。失敗したら False を返す。"""
    print(f'  motor_service  {MOTOR_URI}')
    print(f'  odom_service   {ODOM_URI}')
    print()
    try:
        async with websockets.connect(MOTOR_URI, open_timeout=2.0):
            print('  ✓ motor_service 接続OK')
    except Exception as e:
        print(f'  ✗ motor_service 接続失敗: {e}')
        print('    → launch_all.py が起動しているか確認してください')
        return False
    try:
        async with websockets.connect(ODOM_URI, open_timeout=2.0):
            print('  ✓ odom_service 接続OK')
    except Exception as e:
        print(f'  ✗ odom_service 接続失敗: {e}')
        return False
    print()
    return True
