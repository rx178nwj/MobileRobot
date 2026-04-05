# ROS 2 Docker コンテナ 検証項目

ROS 2 Jazzy コンテナ (`mobile-robot-ros2:jazzy`, `network_mode: host`) の検証項目。  
前提: `edge_services` が起動済みでポート 8001/8002/8003 が LISTEN 状態であること。

---

## 現状の検証カバレッジ

| ID | 内容 | スクリプト | 状態 |
|----|------|-----------|------|
| R1 | イメージビルド確認 | 手動 | **確認済み** (2026-04-05) |
| R2 | コンテナ起動・network_mode:host 確認 | 手動 | **確認済み** (2026-04-05) |
| R3 | colcon ビルド成功確認 | 手動 | **確認済み** (2026-04-05) |
| R4 | ノード起動確認 (3 ノード) | 手動 | **確認済み** (2026-04-05) |
| R5 | /odom 発行確認 (50 Hz) | 手動 | **確認済み** (2026-04-05) |
| R6 | TF ツリー確認 (odom→base_footprint→base_link) | 手動 | **確認済み** (2026-04-05) |
| R7 | /cmd_vel → モーター エンドツーエンド | 手動 | **確認済み** (2026-04-05) |
| R8 | /camera/image_raw/compressed 発行確認 | 手動 | **確認済み** (2026-04-05) |
| R9 | edge_services 再起動後の WebSocket 自動再接続 | 手動 | **確認済み** (2026-04-05) |
| R10 | ノード respawn 動作確認 | 手動 | **確認済み** (2026-04-05) |

---

## 検証項目 詳細

---

### R1: イメージビルド確認

**目的**: Dockerfile が警告・エラーなくビルドでき、必要パッケージがすべてインストールされているか確認する。

**手順**:
```bash
cd ~/MobileRobot/ros2_docker
sudo docker compose build --no-cache 2>&1 | tail -30
sudo docker images | grep mobile-robot-ros2
```

**合否基準**:
- ビルドが `Successfully built` で完了すること
- `ERROR` が出力に含まれないこと

**自動化可否**: 完全自動  
**優先度**: 高 — イメージが壊れると全機能不動

---

### R2: コンテナ起動・network_mode:host 確認

**目的**: コンテナが起動し、`network_mode: host` によりホストの `localhost:8001/8002/8003` にアクセスできることを確認する。

**手順**:
```bash
# コンテナ起動
sudo docker compose run --rm ros2 bash -c "
  python3 -c \"
import asyncio, websockets, json

async def check():
    # 8001 motor: 接続後 stop コマンドを送って ACK を受け取る
    # (motor_service は cmd_vel 受信前はメッセージを送らない)
    async with websockets.connect('ws://localhost:8001', open_timeout=3) as ws:
        await ws.send(json.dumps({'type': 'stop'}))
        ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
        print(f'port 8001 (motor): ACK={ack}')

    # 8002 odom / 8003 camera: 接続後すぐ初回メッセージが届く
    for port, label in [(8002,'odom'), (8003,'camera')]:
        async with websockets.connect(f'ws://localhost:{port}', open_timeout=3) as ws:
            msg = await asyncio.wait_for(ws.recv(), timeout=3.0)
            data = json.loads(msg) if isinstance(msg, str) else json.loads(msg.decode())
            print(f'port {port} ({label}): type={data.get(\\\"type\\\",\\\"?\\\")} OK')

asyncio.run(check())
\"
"
```

**合否基準**:
- `port 8001: OK` / `port 8002: OK` / `port 8003: OK` が出力されること
- `Connection refused` が出ないこと

**自動化可否**: 完全自動  
**優先度**: 高 — host ネットワーク疎通なし → WebSocket ブリッジ全滅

---

### R3: colcon ビルド成功確認

**目的**: `mobile_robot_edge` / `mobile_robot_server` パッケージが警告・エラーなくビルドできるか確認する。

**手順**:
```bash
sudo docker compose run --rm ros2 bash -c "
  cd /ros2_ws &&
  colcon build --packages-select mobile_robot_edge mobile_robot_server \
    --event-handlers console_cohesion+ 2>&1 | tail -20
"
```

**合否基準**:
- `Summary: 2 packages finished` が出力されること
- `Failed` / `error` が含まれないこと

**自動化可否**: 完全自動  
**優先度**: 高 — ビルド失敗 → ノード起動不可

---

### R4: ノード起動確認 (4 ノード)

**目的**: `ws_edge_bringup.launch.py` により所定の 3 ノードが全起動するか確認する。  
※ `ws_camera_bridge` は `server_bringup.launch.py` のみで起動するため対象外。

**前提**: ターミナル A で launch を起動しておく。

```bash
# ターミナル A
sudo docker compose run --rm ros2 bash -c "
  source /ros2_ws/install/setup.bash &&
  ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
"
```

**確認コマンド** (ターミナル B、同一コンテナ内):
```bash
source /ros2_ws/install/setup.bash
ros2 node list
```

**期待される出力**:
```
/robot_state_publisher
/ws_motor_controller
/ws_odometry_publisher
```

**合否基準**:
- 3 ノードすべてがリストに含まれること

**自動化可否**: 完全自動  
**優先度**: 高

---

### R5: /odom 発行確認 (50 Hz)

**目的**: `ws_odometry_publisher` が `edge_services/odometry_service` から受信した odom を `/odom` トピックに 50 Hz で発行しているか確認する。

**確認コマンド**:
```bash
source /ros2_ws/install/setup.bash

# [1] メッセージ内容確認
ros2 topic echo /odom --once

# [2] レート確認
ros2 topic hz /odom --window 100
# 期待: average rate: 50.xxx Hz

# [3] 遅延確認 (stamp と現在時刻の差)
ros2 topic delay /odom
# 期待: < 50 ms
```

**合否基準**:
- `/odom` に `nav_msgs/msg/Odometry` が届くこと
- レート 45〜55 Hz の範囲にあること
- `header.frame_id = "odom"`, `child_frame_id = "base_footprint"` であること

**自動化可否**: 完全自動  
**優先度**: 高 — Nav Stack の入力

---

### R6: TF ツリー確認

**目的**: TF が正しく構成されており、`odom → base_footprint → base_link → camera_optical_frame` のチェーンが切れていないか確認する。

**確認コマンド**:
```bash
source /ros2_ws/install/setup.bash

# [1] TF ツリー全体確認
ros2 run tf2_tools view_frames

# [2] 特定フレーム間の変換確認
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame

# [3] TF 遅延確認
ros2 topic hz /tf
```

**合否基準**:
- `odom → base_footprint` が 50 Hz 近くで更新されること
- `base_footprint → base_link`, `base_link → camera_optical_frame` の static TF が存在すること
- `ros2 run tf2_ros tf2_echo` がタイムアウトしないこと

**自動化可否**: 完全自動  
**優先度**: 高 — TF 欠損 → SLAM / Nav2 動作不可

---

### R7: /cmd_vel → モーター エンドツーエンド

**目的**: `/cmd_vel` を Publish すると `ws_motor_controller` 経由で実際にモーターが動作するか確認する。

**確認コマンド**:
```bash
source /ros2_ws/install/setup.bash

# 前進 0.15 m/s (1秒後に自動停止)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --times 20 --rate 20

# /odom でロボットが前進していることを確認
ros2 topic echo /odom --once
```

**合否基準**:
- コマンド送信後 odom の `pose.position.x` が増加すること
- 停止コマンド (zero twist) 後に `twist.linear.x ≈ 0` になること

**自動化可否**: 完全自動 (ただし実走行が発生)  
**優先度**: 高 — 制御パスの E2E 確認

---

### R8: /camera/image_raw/compressed 発行確認

**目的**: `ws_camera_bridge` が `edge_services/camera_service` からフレームを受信し、  
`/camera/image_raw/compressed` と `/camera/camera_info` を発行しているか確認する。

**確認コマンド**:
```bash
source /ros2_ws/install/setup.bash

# [1] トピック存在確認
ros2 topic list | grep camera

# [2] レート確認 (目標 30 Hz)
ros2 topic hz /camera/image_raw/compressed --window 60
# 期待: average rate: 28〜32 Hz

# [3] メッセージ内容確認
ros2 topic echo /camera/camera_info --once
# width: 640, height: 480, distortion_model: plumb_bob

# [4] CompressedImage フォーマット確認
ros2 topic echo /camera/image_raw/compressed --once | grep format
# format: jpeg
```

**合否基準**:
- `/camera/image_raw/compressed` レート 25〜35 Hz
- `/camera/camera_info` に width=640, height=480 が含まれること
- `format: jpeg` であること

**自動化可否**: 完全自動  
**優先度**: 中 — SLAM / LLM パイプライン入力

---

### R9: edge_services 再起動後の WebSocket 自動再接続

**目的**: `edge_services` が停止・再起動した際に ROS 2 ノードが自動で再接続し、  
データが途切れなく再開されるか確認する。  
`ws_motor_controller` / `ws_odometry_publisher` / `ws_camera_bridge` はいずれも  
`asyncio.sleep(5.0)` でリトライするように実装されている。

**手順**:
1. launch が動作中の状態で `ros2 topic hz /odom` を監視する
2. ホスト側で edge_services を停止: `pkill -f launch_all.py`
3. `/odom` が停止することを確認 (ノードはエラーログを出して再接続待機)
4. ホスト側で edge_services を再起動: `python3 launch_all.py &`
5. 5〜10 秒以内に `/odom` が再開することを確認

**合否基準**:
- 再起動後 60 秒以内に WebSocket が再接続されること  
  (ws_odometry_publisher のリトライ間隔 5s × 最大 12 サイクル)
- ROS 2 ノード自体がクラッシュしないこと (`ros2 node list` で確認)
- edge_services ログに "connection open" が記録されること

**実測値** (2026-04-05): edge_services 再起動後 ~52秒で再接続確認

**自動化可否**: 完全自動  
**優先度**: 中 — 長時間稼働時の安定性

---

### R10: ノード respawn 動作確認

**目的**: launch ファイルで `respawn=True, respawn_delay=5.0` が設定されているノードが  
クラッシュ後に自動復帰するか確認する。

**手順**:
```bash
source /ros2_ws/install/setup.bash

# ws_odometry_publisher を強制終了
ros2 node kill /ws_odometry_publisher
# または
kill $(ros2 node info /ws_odometry_publisher | grep -oP 'PID: \K\d+')

# 5〜10 秒後に再起動を確認
sleep 8 && ros2 node list | grep ws_odometry_publisher
```

**合否基準**:
- 強制終了後 `respawn_delay(5s) + 起動時間` 以内にノードが再リストアップされること
- 再起動後 `/odom` が再び発行されること

**自動化可否**: 完全自動  
**優先度**: 低 — 長時間自律動作の安定性

---

## 優先度マトリクス

| 優先度 | ID | 理由 |
|--------|----|------|
| 高 | R1 | イメージ破損 → 全機能不動 |
| 高 | R2 | host ネットワーク疎通 → WS ブリッジの前提 |
| 高 | R3 | ビルド失敗 → ノード起動不可 |
| 高 | R4 | ノード未起動 → 全機能不動 (ws_edge_bringup: 3ノード / server_bringup: ws_camera_bridge 追加) |
| 高 | R5 | /odom 未配信 → Nav2 / SLAM 動作不可 |
| 高 | R6 | TF 欠損 → SLAM / Nav2 動作不可 |
| 高 | R7 | /cmd_vel E2E → 制御パス確認 |
| 中 | R8 | カメラ画像配信 → SLAM / LLM パイプライン入力 |
| 中 | R9 | WS 自動再接続 → 長時間稼働の安定性 |
| 低 | R10 | respawn → 長時間自律動作の安定性 |

---

## システム構成サマリー

```
[Raspberry Pi — ホスト]
  edge_services/launch_all.py
    ├ motor_service  :8001  (WebSocket)
    ├ odometry_service :8002  (WebSocket)
    └ camera_service :8003  (WebSocket)

[Raspberry Pi — Docker コンテナ, network_mode: host]
  ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
    ├ ws_motor_controller      /cmd_vel → WS:8001
    ├ ws_odometry_publisher    WS:8002 → /odom + TF(odom→base_footprint)
    ├ ws_camera_bridge         WS:8003 → /camera/image_raw/compressed
    └ robot_state_publisher    URDF → TF static(base_footprint→base_link→camera)
```

---

## 全体進捗

| ID | 内容 | 状態 |
|----|------|------|
| R1 | イメージビルド確認 | **確認済み** (2026-04-05) |
| R2 | コンテナ起動・host ネットワーク確認 | **確認済み** (2026-04-05) |
| R3 | colcon ビルド成功確認 | **確認済み** (2026-04-05) |
| R4 | ノード起動確認 (3 ノード) | **確認済み** (2026-04-05) |
| R5 | /odom 発行確認 (50 Hz) | **確認済み** (2026-04-05) |
| R6 | TF ツリー確認 | **確認済み** (2026-04-05) |
| R7 | /cmd_vel → モーター E2E | **確認済み** (2026-04-05) |
| R8 | /camera/image_raw/compressed 発行確認 | **確認済み** (2026-04-05) |
| R9 | edge_services 再起動後 WS 自動再接続 | **確認済み** (2026-04-05) |
| R10 | ノード respawn 動作確認 | **確認済み** (2026-04-05) |
