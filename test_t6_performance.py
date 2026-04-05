#!/usr/bin/env python3
"""
T6 — パフォーマンス・負荷テスト

T6-1: GPU温度 — 連続推論10分間 (90°C未満)
T6-2: VRAMリーク — 長時間稼働でメモリ増加なし
T6-3: CPU使用率 — YOLO+Ollama同時稼働時 80%未満
T6-4: llm_interval=2.0s — 2秒周期連続処理でタイムアウトなし

実行時間: 約10分 (duration_sec=600 をデフォルト)
短縮テスト: python3 test_t6_performance.py --quick  (約3分: 60サイクル)
"""

import sys
import time
import json
import re
import threading
import statistics
import subprocess
import numpy as np
import argparse
import psutil

# ---- 引数 ----
parser = argparse.ArgumentParser()
parser.add_argument('--quick', action='store_true', help='約3分の短縮テスト (60サイクル)')
args = parser.parse_args()

LLM_INTERVAL  = 2.0          # T6-4: 2秒周期
TOTAL_CYCLES  = 60 if args.quick else 300   # quick=60回, 通常=300回 (300×2s≈10分)
PRINT_EVERY   = 10           # N サイクルごとに進捗表示

GPU_TEMP_LIMIT  = 90         # °C
CPU_LIMIT       = 80         # %
VRAM_LEAK_MB    = 200        # この値以上増えたらリーク疑い

print('=' * 70)
print('T6 — パフォーマンス・負荷テスト')
print(f'サイクル数: {TOTAL_CYCLES}  間隔: {LLM_INTERVAL}s  '
      f'目安所要時間: {TOTAL_CYCLES * LLM_INTERVAL / 60:.1f}分')
print('=' * 70)

# -------------------------------------------------------
# 初期化
# -------------------------------------------------------
import torch
from ultralytics import YOLO
import ollama as _ollama

ollama_client = _ollama.Client(host='http://localhost:11434')
MODEL  = 'qwen3.5:9b-nav'

# Ollama を先にウォームアップ（systemd と同順序: Ollama先行→YOLO後ロード）
print('\n[init] Ollamaウォームアップ中 (モデルロード)...')
_t = time.time()
_r = ollama_client.chat(
    model=MODEL,
    messages=[{'role': 'user', 'content': '{"action":"move_forward","reason":"init"}'}],
    think=False,
    options={'num_ctx': 1024},
)
print(f'[init] Ollama warm ({time.time()-_t:.2f}s): {(_r.message.content or "").strip()[:60]}')

print('\n[init] YOLOモデルロード中 (Ollamaウォームアップ後)...')
yolo = YOLO('yolov8s.pt')
yolo.to('cuda')
print('[init] YOLO ready (cuda:0)')

SYSTEM_NAVIGATE = (
    'You are controlling a differential-drive mobile robot. '
    'Reply with ONLY valid JSON, no markdown, no explanation: '
    '{"action": "move_forward"|"turn_left"|"turn_right"|"stop"|"arrived", "reason": "<one sentence>"}'
)

# -------------------------------------------------------
# ユーティリティ
# -------------------------------------------------------
def get_gpu_stats() -> dict:
    """nvidia-smi から温度・VRAM・GPU使用率を取得"""
    try:
        out = subprocess.check_output(
            ['nvidia-smi',
             '--query-gpu=temperature.gpu,memory.used,utilization.gpu',
             '--format=csv,noheader,nounits'],
            timeout=5
        ).decode().strip()
        parts = [p.strip() for p in out.split(',')]
        return {
            'temp_c':    int(parts[0]),
            'vram_used': int(parts[1]),
            'gpu_util':  int(parts[2]),
        }
    except Exception as e:
        return {'temp_c': 0, 'vram_used': 0, 'gpu_util': 0}

def run_pipeline(frame: np.ndarray) -> tuple[str, float]:
    """YOLO → Ollama パイプライン。(action, elapsed_sec) を返す"""
    t0 = time.time()

    # YOLO
    results = yolo(frame, verbose=False, conf=0.3)[0]
    if len(results.boxes) == 0:
        scene = 'No objects detected in the camera view.'
    else:
        items = []
        for box in results.boxes:
            cls_name = results.names[int(box.cls[0])]
            x = float(box.xywhn[0][0])
            y = float(box.xywhn[0][1])
            ph = 'left' if x < 0.33 else ('right' if x > 0.67 else 'center')
            pv = 'close' if y > 0.6 else 'far'
            items.append(f'{cls_name} ({ph},{pv})')
        scene = 'Detected objects: ' + ', '.join(items) + '.'

    # Ollama (native API: think=False 確実動作)
    resp = ollama_client.chat(
        model=MODEL,
        messages=[
            {'role': 'system', 'content': SYSTEM_NAVIGATE},
            {'role': 'user',   'content': f'{scene}\nNavigation goal: "explore". What next?'},
        ],
        think=False,
        options={'num_ctx': 1024, 'temperature': 0.1},
    )
    raw = (resp.message.content or '').strip()

    try:
        result = json.loads(raw)
        action = result.get('action', 'stop')
    except json.JSONDecodeError:
        action = 'stop'

    return action, time.time() - t0

# -------------------------------------------------------
# 測定値コンテナ
# -------------------------------------------------------
latencies    = []
gpu_temps    = []
vram_used    = []
cpu_percents = []
errors       = 0
timeouts     = 0

# VRAMベースライン: モデルロード完了後（ウォームアップ後）に計測
# ※ モデルロード前の 173 MiB を使うとリーク誤判定になるため
gpu0 = get_gpu_stats()
vram_start = gpu0['vram_used']
print(f'\n[init] VRAMベースライン(モデルロード後): {vram_start} MiB  GPU温度: {gpu0["temp_c"]}°C')

# ダミー画像 (黒 640×480)
frame_dummy = np.zeros((480, 640, 3), dtype='uint8')

print(f'\n{"Cycle":>6}  {"Latency":>9}  {"Action":>12}  {"Temp°C":>7}  '
      f'{"VRAM MiB":>9}  {"CPU%":>6}')
print('-' * 65)

start_time = time.time()

for cycle in range(1, TOTAL_CYCLES + 1):
    cycle_start = time.time()

    # CPU使用率 (1サイクル区間)
    cpu = psutil.cpu_percent(interval=None)
    cpu_percents.append(cpu)

    # パイプライン実行
    try:
        action, elapsed = run_pipeline(frame_dummy)
        latencies.append(elapsed)
    except Exception as e:
        errors += 1
        err_msg = str(e)[:50]
        if 'timeout' in err_msg.lower():
            timeouts += 1
        print(f'  ⚠️  cycle {cycle} エラー: {type(e).__name__}: {err_msg}')
        elapsed = LLM_INTERVAL
        action  = 'stop'

    # GPU統計
    gs = get_gpu_stats()
    gpu_temps.append(gs['temp_c'])
    vram_used.append(gs['vram_used'])

    # 進捗表示
    if cycle % PRINT_EVERY == 0 or cycle == 1:
        vram_delta = gs['vram_used'] - vram_start
        print(f'{cycle:>6}  {elapsed:>8.2f}s  {action:>12}  '
              f'{gs["temp_c"]:>6}°C  {gs["vram_used"]:>7} MiB  '
              f'{cpu:>5.1f}%  (ΔVRAM: {vram_delta:+d})')

    # インターバル調整 (2.0s周期維持)
    cycle_elapsed = time.time() - cycle_start
    sleep_time = max(0.0, LLM_INTERVAL - cycle_elapsed)
    if sleep_time > 0:
        time.sleep(sleep_time)

total_time = time.time() - start_time

# -------------------------------------------------------
# 結果集計・合否判定
# -------------------------------------------------------
print('\n' + '=' * 70)
print('T6 テスト結果')
print('=' * 70)

passed = 0
failed = 0

def check(condition, label, detail=''):
    global passed, failed
    if condition:
        passed += 1
        print(f'  ✅ {label}')
    else:
        failed += 1
        print(f'  ❌ {label}' + (f' — {detail}' if detail else ''))

# T6-1: GPU温度
temp_max  = max(gpu_temps)
temp_avg  = statistics.mean(gpu_temps)
print(f'\n[T6-1] GPU温度')
print(f'  最高: {temp_max}°C  平均: {temp_avg:.1f}°C  (上限: {GPU_TEMP_LIMIT}°C)')
check(temp_max < GPU_TEMP_LIMIT,
      f'GPU温度最高 {temp_max}°C < {GPU_TEMP_LIMIT}°C',
      f'max={temp_max}°C')

# T6-2: VRAMリーク
vram_max   = max(vram_used)
vram_final = vram_used[-1]
vram_delta = vram_final - vram_start
print(f'\n[T6-2] VRAMリーク確認')
print(f'  開始: {vram_start} MiB  最終: {vram_final} MiB  '
      f'最大: {vram_max} MiB  変化: {vram_delta:+d} MiB')
check(abs(vram_delta) < VRAM_LEAK_MB,
      f'VRAM変化 {vram_delta:+d} MiB < ±{VRAM_LEAK_MB} MiB (リークなし)',
      f'delta={vram_delta} MiB')

# T6-3: CPU使用率
cpu_max  = max(cpu_percents)
cpu_avg  = statistics.mean(cpu_percents)
print(f'\n[T6-3] CPU使用率')
print(f'  最高: {cpu_max:.1f}%  平均: {cpu_avg:.1f}%  (上限: {CPU_LIMIT}%)')
check(cpu_avg < CPU_LIMIT,
      f'CPU平均使用率 {cpu_avg:.1f}% < {CPU_LIMIT}%',
      f'avg={cpu_avg:.1f}%')

# T6-4: llm_interval=2.0s 安定性
lat_avg    = statistics.mean(latencies) if latencies else 0
lat_max    = max(latencies)            if latencies  else 0
lat_p95    = sorted(latencies)[int(len(latencies)*0.95)] if latencies else 0
error_rate = errors / TOTAL_CYCLES * 100
print(f'\n[T6-4] llm_interval=2.0s 安定性 ({TOTAL_CYCLES}サイクル)')
print(f'  latency avg: {lat_avg:.2f}s  max: {lat_max:.2f}s  p95: {lat_p95:.2f}s')
print(f'  エラー数: {errors}  タイムアウト数: {timeouts}  エラー率: {error_rate:.1f}%')
print(f'  合計稼働時間: {total_time/60:.1f}分')
check(errors == 0,
      f'エラーゼロ ({TOTAL_CYCLES}サイクル連続)',
      f'errors={errors}')
check(lat_max < LLM_INTERVAL * 3,
      f'最大レイテンシ {lat_max:.2f}s < {LLM_INTERVAL*3:.1f}s (interval×3)',
      f'max_lat={lat_max:.2f}s')

# -------------------------------------------------------
# 最終サマリ
# -------------------------------------------------------
print()
print('=' * 70)
total = passed + failed
print(f'T6 結果: {passed}/{total} PASS')
if failed == 0:
    print(f'T6: ✅ 全項目合格 — {TOTAL_CYCLES}サイクル ({total_time/60:.1f}分) 安定稼働')
else:
    print(f'T6: ❌ {failed}項目不合格')

# CSV出力（任意）
csv_path = '/tmp/t6_latency.csv'
with open(csv_path, 'w') as f:
    f.write('cycle,latency_s,gpu_temp_c,vram_mib,cpu_pct\n')
    for i, (lat, tmp, vram, cpu) in enumerate(zip(latencies, gpu_temps, vram_used, cpu_percents), 1):
        f.write(f'{i},{lat:.3f},{tmp},{vram},{cpu:.1f}\n')
print(f'\nCSV: {csv_path}')
print('=' * 70)

sys.exit(0 if failed == 0 else 1)
