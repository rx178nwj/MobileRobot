#!/usr/bin/env python3
"""
Edge Service 動作検証スクリプト
==================================
E3: 安全タイムアウト  — 0.5s無通信後に自動停止
E5: 直進精度         — 1m直進後の誤差 <5cm
E6: 旋回精度         — 360°旋回後の誤差 <5°

Usage:
  cd ~/MobileRobot/edge_services
  python3 verify_e3_e5_e6.py            # インタラクティブ
  python3 verify_e3_e5_e6.py --yes      # 全テスト自動実行
  python3 verify_e3_e5_e6.py --yes e3   # E3のみ自動実行
  python3 verify_e3_e5_e6.py --yes e5   # E5のみ自動実行
  python3 verify_e3_e5_e6.py --yes e6   # E6のみ自動実行
"""

import argparse
import asyncio
import websockets
import json
import math
import sys
import time

MOTOR_URI = 'ws://localhost:8001'
ODOM_URI  = 'ws://localhost:8002'

SEP  = '─' * 58
SEP2 = '═' * 58

# --yes フラグ (argparse で設定)
AUTO_YES = False


def ask(prompt: str) -> bool:
    if AUTO_YES:
        print(f'  {prompt} [y/N]: y  (--yes)')
        return True
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
    """Return the latest odom message, draining any stale buffered messages.

    The publisher sends at 50 Hz (20 ms/msg). Using a 1 ms drain timeout means
    we stop as soon as the queue is empty without catching the next live message.
    Cap at 500 iterations to guarantee termination.
    """
    msg = await asyncio.wait_for(odom_ws.recv(), timeout=2.0)
    for _ in range(500):
        try:
            msg = await asyncio.wait_for(odom_ws.recv(), timeout=0.001)
        except asyncio.TimeoutError:
            break
    return json.loads(msg)


# ─────────────────────────────────────────────────────────────────────────────
# E3: 安全タイムアウト
# ─────────────────────────────────────────────────────────────────────────────

async def test_e3():
    print(f'\n{SEP2}')
    print('  [E3] 安全タイムアウト検証')
    print(f'  仕様: cmd_vel無通信 0.5s 後に自動停止')
    print(SEP2)
    print('  ⚠  モーターが短時間動きます。安全な場所に置いてください。')
    if not ask('開始しますか？'):
        print('  スキップ')
        return

    TIMEOUT_SEC  = 0.5   # motor_service の設定値
    WARMUP_SEC   = 2.5   # ファームウェアPIDがデッドゾーンを超えて速度に達するまで
    CMD_VEL      = 0.5   # m/s (0.3だとデッドゾーク突破前に測定してしまう)

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # --- ステップ1: 10Hzで連続送信してモーターを確実に走行させる ---
        print(f'\n  [1] cmd_vel={CMD_VEL} m/s を {WARMUP_SEC}s 連続送信...')
        t0 = time.time()
        while (time.time() - t0) < WARMUP_SEC:
            await send_cmd_vel(motor_ws, CMD_VEL, 0.0)
            await asyncio.sleep(0.1)

        odom = await get_odom(odom_ws)
        vel_before = abs(odom['twist']['linear'])
        print(f'      送信中の実速度: {vel_before:.3f} m/s')

        # --- ステップ2: 送信を止める ---
        print(f'\n  [2] コマンド送信停止 → {TIMEOUT_SEC}s タイムアウト待機...')
        t_stop = time.time()
        # motor_wsを切断せずにコマンドを送らない (安全タイムアウトを待つ)
        await asyncio.sleep(TIMEOUT_SEC + 0.5)

        # --- ステップ3: 速度確認 ---
        odom = await get_odom(odom_ws)
        vel_after = abs(odom['twist']['linear'])
        elapsed = time.time() - t_stop

        print(f'\n  結果:')
        print(f'    停止前の速度  : {vel_before:.3f} m/s')
        print(f'    {elapsed:.1f}s 後の速度 : {vel_after:.3f} m/s')

        THRESHOLD = 0.05  # m/s
        if vel_after < THRESHOLD:
            print(f'\n  ✓ PASS: {TIMEOUT_SEC}s タイムアウト後に自動停止 ({vel_after:.3f} m/s < {THRESHOLD})')
        else:
            print(f'\n  ✗ FAIL: 停止していません ({vel_after:.3f} m/s ≥ {THRESHOLD})')
            print(f'    → motor_service.py の cmd_vel_timeout を確認してください')


# ─────────────────────────────────────────────────────────────────────────────
# E5: 直進精度
# ─────────────────────────────────────────────────────────────────────────────

async def test_e5():
    print(f'\n{SEP2}')
    print('  [E5] 直進精度検証')
    print(f'  仕様: 1m直進後の誤差 <5cm')
    print(SEP2)
    print('  ⚠  ロボットが前進します。1m以上のスペースを確保してください。')
    if not ask('開始しますか？'):
        print('  スキップ')
        return

    LINEAR_VEL  = 0.15   # m/s (half speed for accuracy)
    TARGET_DIST = 1.0    # m
    STOP_AT     = 0.98   # stop at 98% — at 0.15 m/s coast ≈ 0.02 m (half of 0.3 m/s)
    MAX_TIME    = 20.0   # safety timeout

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # オドメトリをリセット (theta=0 にして向き蓄積をクリア)
        await odom_ws.send(json.dumps({'type': 'reset'}))
        await asyncio.wait_for(odom_ws.recv(), timeout=2.0)  # ack
        await asyncio.sleep(0.1)

        # 初期pose取得
        odom0 = await get_odom(odom_ws)
        x0 = odom0['pose']['x']
        y0 = odom0['pose']['y']
        print(f'\n  開始位置: x={x0:.4f} m, y={y0:.4f} m')
        print(f'  速度: {LINEAR_VEL} m/s  目標: {TARGET_DIST} m (距離ベース停止)')
        print(f'\n  走行中...')

        # 距離ベース停止: 97%到達でコマンド停止、残りはコースト
        t_start = time.time()
        while (time.time() - t_start) < MAX_TIME:
            await send_cmd_vel(motor_ws, LINEAR_VEL, 0.0)
            odom = await get_odom(odom_ws)
            traveled_now = math.sqrt(
                (odom['pose']['x'] - x0)**2 + (odom['pose']['y'] - y0)**2)
            if traveled_now >= TARGET_DIST * STOP_AT:
                break
            await asyncio.sleep(0.05)

        await stop(motor_ws)
        await asyncio.sleep(0.3)  # コースト待ち

        # 終了pose取得
        odom1 = await get_odom(odom_ws)
        x1 = odom1['pose']['x']
        y1 = odom1['pose']['y']

        # 結果計算
        traveled = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        error_m  = abs(traveled - TARGET_DIST)
        lateral  = abs(y1 - y0)  # 横方向誤差

        print(f'\n  結果 (オドメトリ計測):')
        print(f'    目標距離    : {TARGET_DIST:.3f} m')
        print(f'    計測距離    : {traveled:.4f} m')
        print(f'    縦誤差      : {error_m*100:.1f} cm')
        print(f'    横ずれ      : {lateral*100:.1f} cm')

        PASS_CM = 5.0
        if error_m * 100 < PASS_CM:
            print(f'\n  ✓ PASS (オドメトリ): 誤差 {error_m*100:.1f} cm < {PASS_CM} cm')
        else:
            print(f'\n  △ オドメトリ誤差 {error_m*100:.1f} cm ≥ {PASS_CM} cm')

        print(f'\n  📏 物理計測: 実際の移動距離をメジャーで確認してください')
        try:
            actual = float(input('  実測値を入力 (m, スキップはEnter): ').strip() or '0')
            if actual > 0:
                phys_err = abs(actual - TARGET_DIST) * 100
                print(f'    実測誤差: {phys_err:.1f} cm', end='  ')
                print('✓ PASS' if phys_err < PASS_CM else '✗ FAIL')
        except ValueError:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# E6: 旋回精度
# ─────────────────────────────────────────────────────────────────────────────

async def test_e6():
    print(f'\n{SEP2}')
    print('  [E6] 旋回精度検証')
    print(f'  仕様: 360°旋回後の誤差 <5°')
    print(SEP2)
    print('  ⚠  ロボットが360°回転します。十分なスペースを確保してください。')
    if not ask('開始しますか？'):
        print('  スキップ')
        return

    # Minimum reliable angular velocity is ~1.84 rad/s (dead zone = 440 duty = 587 cps).
    # At 2.0 rad/s (638 cps) with ki=2.0, dead zone is overcome in ~0.30 s.
    # Use angle-based stopping: drive until 97% of 360° reached, then coast.
    ANGULAR_VEL  = 2.0           # rad/s  (was 1.0 — too slow for dead zone)
    TARGET_RAD   = 2.0 * math.pi # 360°
    STOP_AT      = 0.984         # 360° - loop_step(5.7°) = 354.3° → final ≈ 360°
    MAX_TIME     = 15.0          # safety timeout

    async with websockets.connect(MOTOR_URI) as motor_ws, \
               websockets.connect(ODOM_URI)  as odom_ws:

        # オドメトリをリセット (前テストの蓄積をクリア)
        await odom_ws.send(json.dumps({'type': 'reset'}))
        await asyncio.wait_for(odom_ws.recv(), timeout=2.0)  # ack
        await asyncio.sleep(0.1)

        # 初期姿勢取得
        odom0 = await get_odom(odom_ws)
        theta0 = odom0['pose']['theta']
        print(f'\n  開始角度: {math.degrees(theta0):.2f}°')
        print(f'  角速度: {ANGULAR_VEL} rad/s  目標: 360° (角度ベース停止)')
        print(f'\n  旋回中...')

        # 角度ベース停止: 97%到達でコマンド停止、残りはコースト
        theta_prev = theta0
        accumulated = 0.0
        t_start = time.time()

        while (time.time() - t_start) < MAX_TIME:
            await send_cmd_vel(motor_ws, 0.0, ANGULAR_VEL)
            odom = await get_odom(odom_ws)
            theta_now = odom['pose']['theta']

            # 角度差分 (-π〜π に正規化)
            d_theta = theta_now - theta_prev
            if d_theta >  math.pi: d_theta -= 2 * math.pi
            if d_theta < -math.pi: d_theta += 2 * math.pi
            accumulated += d_theta
            theta_prev = theta_now

            if abs(accumulated) >= TARGET_RAD * STOP_AT:
                break
            await asyncio.sleep(0.05)

        await stop(motor_ws)
        await asyncio.sleep(0.3)

        # 終了姿勢
        odom1 = await get_odom(odom_ws)
        theta1 = odom1['pose']['theta']

        # 累積角度による誤差
        error_rad = abs(accumulated - TARGET_RAD)
        error_deg = math.degrees(error_rad)

        print(f'\n  結果 (オドメトリ計測):')
        print(f'    目標角度    : 360.00°')
        print(f'    計測角度    : {math.degrees(accumulated):.2f}°')
        print(f'    角度誤差    : {error_deg:.2f}°')

        PASS_DEG = 5.0
        if error_deg < PASS_DEG:
            print(f'\n  ✓ PASS (オドメトリ): 誤差 {error_deg:.2f}° < {PASS_DEG}°')
        else:
            print(f'\n  △ オドメトリ誤差 {error_deg:.2f}° ≥ {PASS_DEG}°')

        print(f'\n  📐 物理確認: ロボットの向きが出発方向と同じか目視確認してください')
        try:
            actual_deg = float(input('  実測角度を入力 (°, スキップはEnter): ').strip() or '0')
            if actual_deg > 0:
                phys_err = abs(actual_deg - 360.0)
                print(f'    実測誤差: {phys_err:.1f}°', end='  ')
                print('✓ PASS' if phys_err < PASS_DEG else '✗ FAIL')
        except ValueError:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

async def main(run_tests: list):
    print()
    print(SEP2)
    print('  Edge Service 動作検証  E3 / E5 / E6')
    print(SEP2)
    print()
    print('  motor_service  ws://localhost:8001')
    print('  odom_service   ws://localhost:8002')
    print()

    # 接続確認
    try:
        async with websockets.connect(MOTOR_URI, open_timeout=2.0):
            print('  ✓ motor_service 接続OK')
    except Exception as e:
        print(f'  ✗ motor_service 接続失敗: {e}')
        print('    → launch_all.py が起動しているか確認してください')
        return

    try:
        async with websockets.connect(ODOM_URI, open_timeout=2.0):
            print('  ✓ odom_service 接続OK')
    except Exception as e:
        print(f'  ✗ odom_service 接続失敗: {e}')
        return

    print()
    if 'e3' in run_tests: await test_e3()
    if 'e5' in run_tests: await test_e5()
    if 'e6' in run_tests: await test_e6()

    print()
    print(SEP2)
    print('  検証完了')
    print(SEP2)
    print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Edge service verification E3/E5/E6')
    parser.add_argument('--yes', action='store_true',
                        help='確認プロンプトをスキップして自動実行')
    parser.add_argument('tests', nargs='*', default=['e3', 'e5', 'e6'],
                        help='実行するテスト (例: e3  e3 e5  e6, デフォルト: 全て)')
    args = parser.parse_args()

    if args.yes:
        import builtins
        _orig_input = builtins.input
        builtins.input = lambda prompt='': (print(prompt + 'y  (--yes)'), 'y')[1]

    run_tests = [t.lower() for t in args.tests]
    asyncio.run(main(run_tests))
