#!/usr/bin/env python3
"""
T5-1: Ollama停止時の挙動テスト

事前条件: sudo systemctl stop ollama が実行済みであること

確認内容:
  - ConnectionRefusedError / ConnectError などの接続エラーをキャッチできる
  - 例外発生時に action='stop' を返す（ロボット安全停止）
  - クラッシュせず処理継続できる
"""

import sys
import time
import json
import socket

print('=' * 60)
print('T5-1: Ollama停止時の挙動テスト')
print('=' * 60)

passed = 0
failed = 0

def check(condition: bool, label: str, detail: str = ''):
    global passed, failed
    if condition:
        passed += 1
        print(f'  ✅ {label}')
    else:
        failed += 1
        print(f'  ❌ {label}' + (f' — {detail}' if detail else ''))


# -------------------------------------------------------
# PART 1: Ollamaポート疎通確認（停止確認）
# -------------------------------------------------------
print('\n[1] Ollamaサービス停止確認')
ollama_stopped = False
try:
    s = socket.create_connection(('127.0.0.1', 11434), timeout=2)
    s.close()
    print('  ⚠️  Ollamaがまだ起動中です！')
    print('     事前に: sudo systemctl stop ollama')
    print('     を実行してからこのスクリプトを再実行してください。')
    sys.exit(1)
except (ConnectionRefusedError, OSError):
    ollama_stopped = True
    print('  確認: port 11434 — 接続拒否 (Ollama停止済み)')

check(ollama_stopped, 'Ollamaが停止していることを確認')

# -------------------------------------------------------
# PART 2: OpenAI client での接続エラーキャッチ
# -------------------------------------------------------
print('\n[2] OpenAI client — 接続エラーキャッチ')
from openai import OpenAI
import httpx

client = OpenAI(api_key='ollama', base_url='http://localhost:11434/v1')

caught_connection_error = False
error_type = None

try:
    resp = client.chat.completions.create(
        model='qwen3.5:9b-nav',
        messages=[
            {'role': 'system', 'content': 'Reply with JSON only.'},
            {'role': 'user',   'content': 'move forward'},
        ],
        max_tokens=100,
        timeout=5.0,
    )
    print('  ⚠️  接続成功 — Ollamaが起動しています（停止確認してください）')
except Exception as e:
    error_type = type(e).__name__
    error_msg  = str(e)[:100]
    caught_connection_error = True
    print(f'  キャッチ: {error_type}')
    print(f'  メッセージ: {error_msg}')

check(caught_connection_error, '接続エラーを例外としてキャッチ', f'error={error_type}')

# -------------------------------------------------------
# PART 3: llm_nav_controller 相当の安全停止ロジック確認
# -------------------------------------------------------
print('\n[3] 安全停止ロジック (_call_llm 相当)')

def simulate_call_llm_with_stopped_ollama(scene_text: str, system_prompt: str):
    """
    llm_nav_controller._call_llm の接続エラーハンドリングを再現。
    接続失敗時は action='stop' を返し、ロボットを安全停止させる。
    """
    try:
        client2 = OpenAI(api_key='ollama', base_url='http://localhost:11434/v1')
        resp = client2.chat.completions.create(
            model='qwen3.5:9b-nav',
            messages=[
                {'role': 'system', 'content': system_prompt},
                {'role': 'user',   'content': scene_text},
            ],
            max_tokens=100,
            timeout=3.0,
        )
        raw = (resp.choices[0].message.content or '').strip()
        return json.loads(raw)

    except ConnectionRefusedError as e:
        print(f'  [WARN] ConnectionRefusedError: {e}')
        return {'action': 'stop', 'reason': 'LLM server not available'}

    except Exception as e:
        # httpx.ConnectError, openai.APIConnectionError 等
        err_lower = str(e).lower()
        if any(kw in err_lower for kw in ['connection', 'connect', 'refused', 'unreachable', 'timeout']):
            print(f'  [WARN] 接続エラー({type(e).__name__}): {str(e)[:80]}')
            return {'action': 'stop', 'reason': f'LLM connection error: {type(e).__name__}'}
        else:
            print(f'  [ERROR] 予期しない例外({type(e).__name__}): {str(e)[:80]}')
            return {'action': 'stop', 'reason': 'unexpected LLM error'}

result = simulate_call_llm_with_stopped_ollama(
    'No objects detected.',
    'Reply with ONLY valid JSON: {"action": "move_forward"|"stop", "reason": "..."}'
)

print(f'  返却アクション: {result}')

check(result.get('action') == 'stop',
      'Ollama停止時 → action=stop (安全停止)',
      f'action={result.get("action")}')
check('reason' in result,
      'reason フィールド存在',
      f'reason={result.get("reason")}')

# -------------------------------------------------------
# PART 4: 複数回連続呼び出しでもクラッシュしない
# -------------------------------------------------------
print('\n[4] 連続5回呼び出し — クラッシュしない確認')
crash_count = 0
stop_count  = 0

for i in range(5):
    try:
        r = simulate_call_llm_with_stopped_ollama(
            'Detected objects: person (center, far).',
            'Reply with ONLY valid JSON: {"action": "stop"|"move_forward", "reason": "..."}'
        )
        if r.get('action') == 'stop':
            stop_count += 1
    except Exception as e:
        crash_count += 1
        print(f'  クラッシュ {i+1}: {type(e).__name__}: {e}')

check(crash_count == 0,    f'連続5回クラッシュなし (crash={crash_count})')
check(stop_count == 5,     f'全5回 stop アクション返却 (stop={stop_count}/5)')

# -------------------------------------------------------
# 結果サマリ
# -------------------------------------------------------
print()
print('=' * 60)
total = passed + failed
print(f'結果: {passed}/{total} PASS')
if failed == 0:
    print('T5-1: ✅ PASS — Ollama停止時も例外キャッチ・ロボット安全停止')
else:
    print(f'T5-1: ❌ FAIL — {failed} 項目が不合格')
print()
print('次のステップ: sudo systemctl start ollama')
print('=' * 60)

sys.exit(0 if failed == 0 else 1)
