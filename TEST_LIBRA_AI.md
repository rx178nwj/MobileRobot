# Libra AI System Test Plan

対象: Libra — Ubuntu 24.04 Host System (NVIDIA RTX 4060 Laptop 8GB / CUDA 13.0)

AIパイプライン:
```
/camera/image_raw → YOLO yolov8s.pt (cuda:0) → scene text → Ollama qwen3.5:9b → JSON action
```

---

## T1 — 環境・インストール確認

| ID | テスト項目 | 確認コマンド | 合格基準 | 結果 |
|----|-----------|-------------|---------|------|
| T1-1 | ultralytics インストール | `python3 -c "from ultralytics import YOLO; print('OK')"` | OK表示 | ✅ PASS |
| T1-2 | openai パッケージ | `python3 -c "from openai import OpenAI; print('OK')"` | OK表示 | ✅ PASS |
| T1-3 | CUDA認識 | `python3 -c "import torch; print(torch.cuda.is_available())"` | `True` | ✅ PASS |
| T1-4 | GPU情報 | `nvidia-smi --query-gpu=name,memory.total --format=csv` | RTX 4060 / 8188MiB | ✅ PASS |
| T1-5 | Ollamaサービス状態 | `systemctl status ollama` | `active (running)` | ✅ PASS |
| T1-6 | Ollama外部公開 | `ss -tlnp \| grep 11434` | `*:11434` | ✅ PASS |

---

## T2 — YOLO 単体テスト

| ID | テスト項目 | 内容 | 合格基準 | 結果 |
|----|-----------|------|---------|------|
| T2-1 | モデルロード | `YOLO('yolov8s.pt')` 起動時間計測 | 3秒以内 | ✅ PASS (2.02s) |
| T2-2 | GPU推論デバイス確認 | `next(model.model.parameters()).device` | `cuda:0` | ✅ PASS ※要 `.to('cuda')` 明示 |
| T2-3 | ダミー画像推論 | 640×480 黒画像で推論 | エラーなし・完走 | ✅ PASS |
| T2-4 | 推論速度 | 640×480 JPEG 1枚の処理時間 | 50ms以内（GPU） | ✅ PASS (avg 7.1ms) |
| T2-5 | 人物検出精度 | 人が写った画像で検出 | `person` クラス検出 | ✅ PASS (4 persons + bus) |
| T2-6 | scene text生成 | `_extract_scene_description()` の出力形式 | `"Detected objects: person (center, far)."` | ✅ PASS |
| T2-7 | 物体なし処理 | 空白画像で推論 | `"No objects detected..."` 返却 | ✅ PASS |
| T2-8 | 信頼度閾値 | `conf=0.3` での検出数 | 誤検出が少ない | ✅ PASS (0 false positives) |

### T2 テスト実行スクリプト

```python
# test_yolo.py
import time
import numpy as np
from ultralytics import YOLO

model = YOLO('yolov8s.pt')
print(f'device: {next(model.model.parameters()).device}')

# T2-3 / T2-4: ダミー画像推論 + 速度
dummy = np.zeros((480, 640, 3), dtype='uint8')
start = time.time()
results = model(dummy, verbose=False, conf=0.3)[0]
elapsed = (time.time() - start) * 1000
print(f'推論時間: {elapsed:.1f}ms')

# T2-7: 物体なし
if len(results.boxes) == 0:
    print('No objects detected in the camera view.')
else:
    items = []
    for box in results.boxes:
        cls_name = results.names[int(box.cls[0])]
        x_center = float(box.xywhn[0][0])
        y_center = float(box.xywhn[0][1])
        pos_h = 'left' if x_center < 0.33 else ('right' if x_center > 0.67 else 'center')
        pos_v = 'close' if y_center > 0.6 else 'far'
        items.append(f'{cls_name} ({pos_h}, {pos_v})')
    print('Detected objects: ' + ', '.join(items) + '.')
```

```bash
python3 test_yolo.py
```

---

## T3 — Ollama / qwen3.5:9b 単体テスト

| ID | テスト項目 | 内容 | 合格基準 | 結果 |
|----|-----------|------|---------|------|
| T3-1 | API疎通（ローカル） | `curl localhost:11434/api/tags` | `qwen3.5:9b` 表示 | ✅ PASS (qwen3.5:9b-nav, qwen3.5:9b, gemma4:e2b) |
| T3-2 | API疎通（外部） | 他PCから `curl 192.168.10.35:11434/api/tags` | `qwen3.5:9b` 表示 | ✅ PASS |
| T3-3 | モデルロード時間 | 初回推論のレスポンス時間 | 10秒以内 | ✅ PASS (1.75s) |
| T3-4 | navigate応答形式 | navigate用プロンプト送信 | `{"action": "...", "reason": "..."}` | ✅ PASS (action=stop, 1.77s) |
| T3-5 | track応答形式 | track用プロンプト送信 | `{"action":..., "person_detected":..., "person_position":...}` | ✅ PASS (turn_left, left, 1.64s) |
| T3-6 | explore応答形式 | explore用プロンプト送信 | `{"action":..., "observation":...}` | ✅ PASS (move_forward, 1.55s) |
| T3-7 | thinking除去 | `<think>...</think>` ブロックの除去処理 | JSONのみ返却 | ✅ PASS (think=false: contentに直接JSON / think=true: thinkingフィールドに分離) |
| T3-8 | 推論速度 | scene text → JSON action の応答時間 | 2秒以内 | ✅ PASS (avg 1.26s, min 0.94s, max 1.55s) |
| T3-9 | VRAM使用量 | `ollama ps` + `nvidia-smi` | 8GB以内 | ✅ PASS (6059 MiB / 8188 MiB) |

### T3 テスト実行スクリプト

```python
# test_ollama.py
import time
import json
import re
from openai import OpenAI

client = OpenAI(api_key='ollama', base_url='http://localhost:11434/v1')
MODEL = 'qwen3.5:9b'

SYSTEM_NAVIGATE = (
    'You are controlling a differential-drive mobile robot. '
    'Reply with ONLY valid JSON, no markdown, no explanation: '
    '{"action": "move_forward"|"turn_left"|"turn_right"|"stop"|"arrived", "reason": "<one sentence>"}'
)

scene = 'Detected objects: person (center, far).'
prompt = f'/no_think {scene}\n\nNavigation goal: "go to the person". What should I do next?'

start = time.time()
resp = client.chat.completions.create(
    model=MODEL,
    messages=[
        {'role': 'system', 'content': SYSTEM_NAVIGATE},
        {'role': 'user',   'content': prompt},
    ],
    max_tokens=200,
    temperature=0.1,
)
elapsed = time.time() - start

raw = (resp.choices[0].message.content or '').strip()
# thinking除去
if '<think>' in raw:
    raw = re.sub(r'<think>.*?</think>', '', raw, flags=re.DOTALL).strip()

print(f'応答時間: {elapsed:.2f}s')
print(f'raw: {raw}')
result = json.loads(raw)
print(f'action: {result["action"]}')
print(f'VRAM: ollama ps で確認')
```

```bash
python3 test_ollama.py
ollama ps
nvidia-smi --query-gpu=memory.used,memory.free --format=csv
```

---

## T4 — YOLO → Ollama 統合パイプラインテスト

| ID | テスト項目 | 内容 | 合格基準 | 結果 |
|----|-----------|------|---------|------|
| T4-1 | パイプライン直列動作 | YOLO scene text → Ollama → JSON まで一気通貫 | エラーなし | ✅ PASS (1.35s, action=move_forward) |
| T4-2 | navigate end-to-end | 人物画像 → `"go to the person"` → action | `move_forward` or `turn_left/right` | ✅ PASS (action=turn_left, 1.68s) |
| T4-3 | track end-to-end | 人物左側画像 → track mode | `{"person_position": "left", "action": "turn_left"}` | ✅ PASS (turn_left, detected=True, pos=left) |
| T4-4 | explore end-to-end | 空間画像 → explore mode | `{"action": "move_forward", "observation": "..."}` | ✅ PASS (move_forward, 1.36s) |
| T4-5 | 全体レイテンシ | 画像入力 → JSON出力までの時間 | 3秒以内（YOLO+Ollama合計） | ✅ PASS (avg 1.53s, min 1.34s, max 1.68s) |
| T4-6 | 連続処理（10回） | 同一画像を10回繰り返し処理 | 全回エラーなし・安定したaction | ✅ PASS (0 errors, avg 1.63s) |
| T4-7 | VRAM同時使用 | YOLO + Ollama 同時ロード時のVRAM | 8GB超過しない | ✅ PASS (6298 MiB / 8188 MiB) |

### T4 テスト実行スクリプト

```python
# test_pipeline.py
import time
import json
import re
import numpy as np
from ultralytics import YOLO
from openai import OpenAI

yolo = YOLO('yolov8s.pt')
client = OpenAI(api_key='ollama', base_url='http://localhost:11434/v1')
MODEL = 'qwen3.5:9b'

SYSTEM_NAVIGATE = (
    'You are controlling a differential-drive mobile robot. '
    'Reply with ONLY valid JSON, no markdown, no explanation: '
    '{"action": "move_forward"|"turn_left"|"turn_right"|"stop"|"arrived", "reason": "<one sentence>"}'
)

def run_pipeline(frame: np.ndarray, goal: str) -> dict:
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
            items.append(f'{cls_name} ({ph}, {pv})')
        scene = 'Detected objects: ' + ', '.join(items) + '.'
    t_yolo = time.time() - t0

    # Ollama
    prompt = f'/no_think {scene}\n\n{goal}'
    resp = client.chat.completions.create(
        model=MODEL,
        messages=[
            {'role': 'system', 'content': SYSTEM_NAVIGATE},
            {'role': 'user',   'content': prompt},
        ],
        max_tokens=200, temperature=0.1,
    )
    raw = (resp.choices[0].message.content or '').strip()
    if '<think>' in raw:
        raw = re.sub(r'<think>.*?</think>', '', raw, flags=re.DOTALL).strip()
    t_total = time.time() - t0

    result = json.loads(raw)
    print(f'scene : {scene}')
    print(f'action: {result["action"]}')
    print(f'YOLO  : {t_yolo*1000:.0f}ms  /  Total: {t_total:.2f}s')
    return result

# T4-6: 連続10回テスト
frame = np.zeros((480, 640, 3), dtype='uint8')
print('=== 連続10回テスト ===')
for i in range(10):
    print(f'--- {i+1}/10 ---')
    run_pipeline(frame, 'Navigation goal: "explore the room". What should I do next?')
```

```bash
python3 test_pipeline.py
# 実行中に別ターミナルでVRAM監視
nvidia-smi --query-gpu=memory.used,temperature.gpu --format=csv -l 2
```

---

## T5 — 異常系・エラーハンドリングテスト

| ID | テスト項目 | 内容 | 合格基準 | 結果 |
|----|-----------|------|---------|------|
| T5-1 | Ollama停止時の挙動 | `systemctl stop ollama` 後に推論 | 例外キャッチ・ロボット停止 | |
| T5-2 | 不正JSON応答 | Ollama応答が壊れた場合 | `json.JSONDecodeError` キャッチ・停止維持 | |
| T5-3 | カメラフレームなし | `frame=None` で推論呼び出し | 処理スキップ・クラッシュなし | |
| T5-4 | GPU OOM | VRAM不足状態でYOLO推論 | エラーログ出力・継続動作 | |
| T5-5 | Ollama keepalive切れ | モデルアンロード後の再呼び出し | 自動再ロード・動作継続 | |

```bash
# T5-1: Ollama停止テスト
sudo systemctl stop ollama
python3 test_ollama.py   # → ConnectionRefusedError キャッチ確認
sudo systemctl start ollama

# T5-5: keepalive切れ確認
ollama ps   # UNTILが切れた後に test_ollama.py を実行
```

---

## T6 — パフォーマンス・負荷テスト

| ID | テスト項目 | 内容 | 合格基準 | 結果 |
|----|-----------|------|---------|------|
| T6-1 | GPU温度 | 連続推論10分間のGPU温度 | 90°C未満 | |
| T6-2 | VRAMリーク | 長時間稼働時のVRAM変化 | メモリ増加なし | |
| T6-3 | CPU使用率 | YOLO+Ollama同時稼働時 | 80%未満 | |
| T6-4 | llm_interval=2.0s | 2秒周期での連続処理 | タイムアウトなし | |

```bash
# T6-1 / T6-2: GPU監視（別ターミナルで実行）
nvidia-smi --query-gpu=temperature.gpu,memory.used,utilization.gpu --format=csv -l 5
```

---

## テスト実施優先度

```
Phase 1（必須）: T1全項目 → T2-1〜T2-4 → T3-1〜T3-4 → T4-1 → T4-5
Phase 2（統合）: T4-2〜T4-7 → T5全項目
Phase 3（負荷）: T6全項目
```

---

## 結果記録

| 実施日 | Phase | 合格数 / 総数 | 備考 |
|--------|-------|-------------|------|
| 2026-04-04 | Phase 1 (T1) | 6 / 6 | T1全項目合格 |
| 2026-04-04 | Phase 1 (T2) | 8 / 8 | T2全項目合格 / 要注意: YOLO GPU使用には `.to('cuda')` 明示が必要 |
| 2026-04-04 | YOLO+Ollama共存 | ✅ | YOLO(GPU)+Ollama同時動作確認 / think=false必須 / 合計1.2〜1.9s |
| 2026-04-04 | Phase 1 (T3) | 9 / 9 | T3全項目合格 / モデル: qwen3.5:9b-nav / 平均1.26s |
| 2026-04-04 | Phase 2 (T4) | 7 / 7 | T4全項目合格 / avg 1.53s(T4-5) / 10回連続エラーなし(T4-6) |
| | Phase 2 | / 12 | |
| | Phase 3 | / 4  | |

---

*Updated: 2026-04-04*
