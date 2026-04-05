#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト — E3
=====================================
E3: 安全タイムアウト — 0.5s 無通信後に自動停止

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e3.py
  python3 verify_e3.py --yes   # 確認プロンプトをスキップ
"""

import argparse
import asyncio
import time
import websockets

from _verify_common import (
    MOTOR_URI, ODOM_URI, SEP, SEP2,
    ask, send_cmd_vel, get_odom, check_services,
)


async def test_e3():
    print(f'\n{SEP2}')
    print('  [E3] 安全タイムアウト検証')
    print(f'  仕様: cmd_vel 無通信 0.5s 後に自動停止')
    print(SEP2)
    print('  ⚠  モーターが短時間動きます。安全な場所に置いてください。')
    if not ask('開始しますか？'):
        print('  スキップ')
        return

    TIMEOUT_SEC = 0.5   # motor_service の設定値
    WARMUP_SEC  = 2.5   # ファームウェア PID がデッドゾーンを超えて速度に達するまで
    CMD_VEL     = 0.5   # m/s

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # ステップ1: 連続送信してモーターを定常速度に
        print(f'\n  [1] cmd_vel={CMD_VEL} m/s を {WARMUP_SEC}s 連続送信...')
        t0 = time.time()
        while (time.time() - t0) < WARMUP_SEC:
            await send_cmd_vel(motor_ws, CMD_VEL, 0.0)
            await asyncio.sleep(0.1)

        odom = await get_odom(odom_ws)
        vel_before = abs(odom['twist']['linear'])
        print(f'      送信中の実速度: {vel_before:.3f} m/s')

        # ステップ2: 送信停止 → タイムアウト待機
        print(f'\n  [2] コマンド送信停止 → {TIMEOUT_SEC}s タイムアウト待機...')
        t_stop = time.time()
        await asyncio.sleep(TIMEOUT_SEC + 1.0)

        # ステップ3: 速度確認
        odom = await get_odom(odom_ws)
        vel_after = abs(odom['twist']['linear'])
        elapsed = time.time() - t_stop

        print(f'\n  結果:')
        print(f'    停止前の速度  : {vel_before:.3f} m/s')
        print(f'    {elapsed:.1f}s 後の速度 : {vel_after:.3f} m/s')

        THRESHOLD = 0.05
        if vel_after < THRESHOLD:
            print(f'\n  ✓ PASS: {TIMEOUT_SEC}s タイムアウト後に自動停止 ({vel_after:.3f} m/s < {THRESHOLD})')
        else:
            print(f'\n  ✗ FAIL: 停止していません ({vel_after:.3f} m/s ≥ {THRESHOLD})')
            print(f'    → motor_service.py の cmd_vel_timeout を確認してください')


async def main():
    print()
    print(SEP2)
    print('  Edge Service 動作検証  E3')
    print(SEP2)
    print()
    if not await check_services():
        return
    await test_e3()
    print()
    print(SEP2)
    print('  検証完了')
    print(SEP2)
    print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Edge service verification E3')
    parser.add_argument('--yes', action='store_true', help='確認プロンプトをスキップして自動実行')
    args = parser.parse_args()

    if args.yes:
        import builtins
        builtins.input = lambda prompt='': (print(prompt + 'y  (--yes)'), 'y')[1]

    asyncio.run(main())
