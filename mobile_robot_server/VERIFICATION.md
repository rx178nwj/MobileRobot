# Nav2 Navigation Stack 検証項目

`lifecycle_manager_navigation` が管理する 9 ノードを対象とした動作検証。  
`server_bringup.launch.py` で起動するサーバーサイド (Ubuntu Server) の検証。

---

## 前提条件

| 項目 | 要件 |
|------|------|
| YDLIDAR T-mini Plus | `/dev/ttyUSB0` に接続済み |
| Raspberry Pi (edge) | `launch_all.py` + `ws_edge_bringup.launch.py` 起動済み |
| slam_toolbox | `/map` と TF `map → odom` を配信中 |
| `/odom` | ws_odometry_publisher から 50Hz で配信中 |
| ROS_DOMAIN_ID | Pi と Server で一致 (デフォルト: 0) |

```bash
# 起動コマンド (Ubuntu Server)
ros2 launch mobile_robot_server server_bringup.launch.py
```

---

## lifecycle_manager_navigation 管理ノード (9 nodes)

| # | ノード名 | 役割 |
|---|---------|------|
| 1 | controller_server | RegulatedPurePursuit — 速度指令生成 (10Hz) |
| 2 | smoother_server | SimpleSmoother — パス平滑化 |
| 3 | planner_server | NavFn (A*) — グローバルパスプランニング |
| 4 | route_server | ルートグラフナビゲーション (Jazzy 新機能) |
| 5 | behavior_server | spin / backup / wait — 回復行動 |
| 6 | bt_navigator | BT Navigator — navigate_to_pose / navigate_through_poses |
| 7 | waypoint_follower | 複数ウェイポイント順次訪問 |
| 8 | velocity_smoother | 速度平滑化 (20Hz) — cmd_vel_nav → cmd_vel_smoothed |
| 9 | collision_monitor | LiDAR 障害物検出 → 緊急停止 — cmd_vel_smoothed → cmd_vel |

**速度指令パイプライン**:
```
/goal_pose
  → bt_navigator
    → planner_server  →  /plan (グローバルパス)
    → controller_server  →  /cmd_vel_nav
      → velocity_smoother  →  /cmd_vel_smoothed
        → collision_monitor  →  /cmd_vel
          → ws_motor_controller → edge_services → モーター
```

---

## 現状の検証カバレッジ

| ID | 内容 | 状態 |
|----|------|------|
| N1 | 9ノード起動 & lifecycle active 確認 | **確認済み** (2026-04-05) |
| N2 | 必須トピック・TF チェーン確認 | 未実施 |
| N3 | グローバルコストマップ配信確認 | 未実施 |
| N4 | ローカルコストマップ配信確認 | 未実施 |
| N5 | navigate_to_pose アクションサーバー確認 | 未実施 |
| N6 | グローバルパスプランニング確認 | 未実施 |
| N7 | ナビゲーション E2E (近距離目標到達) | 未実施 |
| N8 | velocity_smoother パイプライン確認 | 未実施 |
| N9 | collision_monitor 緊急停止確認 | 未実施 |
| N10 | 回復行動 (spin / backup) 確認 | 未実施 |
| N11 | waypoint_follower 複数地点訪問 | 未実施 |
| N12 | lifecycle bond — ノード障害時の再起動 | 未実施 |

---

## 検証項目 詳細

---

### N1: 9ノード起動 & lifecycle active 確認

**目的**: lifecycle_manager_navigation が管理する 9 ノードがすべて `active` 状態で起動しているか確認する。

```bash
# ノードリスト確認
ros2 node list | grep -E "controller|smoother|planner|route|behavior|bt_nav|waypoint|velocity|collision|lifecycle"

# 各ノードのライフサイクル状態確認
for node in controller_server smoother_server planner_server route_server \
            behavior_server bt_navigator waypoint_follower \
            velocity_smoother collision_monitor; do
  STATE=$(ros2 lifecycle get /$node 2>/dev/null | head -1)
  echo "/$node: $STATE"
done
```

**合否基準**:
- 9 ノードすべてが `ros2 node list` に出現すること
- 各ノードのライフサイクル状態が `active` であること
- `lifecycle_manager_navigation` ログに `"All nodes are active"` が出力されること

**自動化可否**: 完全自動  
**優先度**: 高 — ここが FAIL なら全 N-テスト不可

---

### N2: 必須トピック・TF チェーン確認

**目的**: Nav2 が必要とするすべての入力トピックと TF が届いているか確認する。

```bash
# 必須トピック確認
ros2 topic list | grep -E "^/(map|odom|scan|tf|tf_static)$"

# TF チェーン確認: map → odom → base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
# 期待: 変換が返ること (map → odom: slam_toolbox / odom → base_footprint: ws_odom_pub)

# レート確認
ros2 topic hz /map   --window 5   # 期待: ~0.33 Hz (3s 更新)
ros2 topic hz /odom  --window 100 # 期待: ~50 Hz
ros2 topic hz /scan  --window 20  # 期待: ~10 Hz (YDLIDAR T-mini Plus)
```

**合否基準**:
- `/map` / `/odom` / `/scan` / `/tf` / `/tf_static` が存在すること
- `tf2_echo map base_footprint` がタイムアウトしないこと
- `/scan` レートが 8〜12 Hz の範囲にあること

**自動化可否**: 完全自動  
**優先度**: 高 — Nav2 の前提条件

---

### N3: グローバルコストマップ配信確認

**目的**: `/global_costmap/costmap` が `/map` を受信して正しく生成されているか確認する。

```bash
# トピック確認
ros2 topic list | grep global_costmap

# コストマップ情報確認
ros2 topic echo /global_costmap/costmap --once \
  | grep -E 'resolution|width|height|frame_id'
# 期待: resolution=0.05, frame_id=map

# 更新レート確認 (設定: 1.0 Hz)
ros2 topic hz /global_costmap/costmap --window 10
```

**合否基準**:
- `/global_costmap/costmap` に `nav2_msgs/msg/Costmap` または `nav_msgs/msg/OccupancyGrid` が届くこと
- `resolution: 0.05` / `frame_id: map` であること
- 更新レート 0.5〜2.0 Hz の範囲にあること

**自動化可否**: 完全自動 (/map 受信後)  
**優先度**: 高 — グローバルパスプランニングの基盤

---

### N4: ローカルコストマップ配信確認

**目的**: `/local_costmap/costmap` が YDLIDAR スキャン (/scan) を受信して動的に更新されているか確認する。

```bash
ros2 topic echo /local_costmap/costmap --once \
  | grep -E 'resolution|width|height|frame_id'
# 期待: resolution=0.05, frame_id=odom, width=60 (3m / 0.05m)

ros2 topic hz /local_costmap/costmap --window 20
# 期待: ~2 Hz (publish_frequency=2.0)

# 障害物レイヤー: /scan が反映されているか (Foxglove で視覚確認)
```

**合否基準**:
- `frame_id: odom` / `resolution: 0.05` / `width ≈ 60` であること
- 更新レート 1.5〜3.0 Hz の範囲にあること
- ロボット前方に実物を置いたとき costmap に障害物セルが追加されること (Foxglove 目視)

**自動化可否**: 部分自動 (障害物テストは手動)  
**優先度**: 高 — ローカル障害物回避の基盤

---

### N5: navigate_to_pose アクションサーバー確認

**目的**: `bt_navigator` が提供する `/navigate_to_pose` アクションサーバーが応答するか確認する。

```bash
# アクションサーバー存在確認
ros2 action list | grep navigate

# アクションサーバー情報確認
ros2 action info /navigate_to_pose
# 期待: Action servers: /bt_navigator

# ドライラン: 現在地付近にゴールを送り、アクションが受け付けられるか
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
   orientation: {w: 1.0}}}}" --feedback
```

**合否基準**:
- `/navigate_to_pose` と `/navigate_through_poses` がアクションリストに存在すること
- ゴール送信後に `Accepted` または `Executing` ステータスが返ること
- エラーなくアクションが開始されること

**自動化可否**: 完全自動  
**優先度**: 高 — ナビゲーション E2E の入口

---

### N6: グローバルパスプランニング確認

**目的**: `planner_server` が現在位置から目標点までのグローバルパス (/plan) を生成できるか確認する。

```bash
# /plan トピックが配信されているか
ros2 topic echo /plan --once | grep -E 'frame_id|poses'

# サービスを直接呼び出して経路計画を確認
ros2 service call /planner_server/plan_path nav2_msgs/srv/GetPath \
  "{start: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}, \
    goal:  {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  2>/dev/null | head -20
```

**合否基準**:
- `/plan` トピックに `nav_msgs/Path` が届くこと
- 経路が空でないこと (`poses` 配列が非空)
- `frame_id: map` であること

**自動化可否**: 完全自動 (/map 受信後)  
**優先度**: 高 — パスプランニングは Nav2 の核心機能

---

### N7: ナビゲーション E2E (近距離目標到達)

**目的**: `/goal_pose` にゴールを送信し、ロボットが実際に到達するか確認する。  
**安全**: 障害物のない 0.5m 以内の短距離ゴールを使用すること。

```bash
# 現在の odom 位置を確認してからゴールを送信
ros2 topic echo /odom --once | grep -A3 'position:'

# x=0.5m 前方 (map フレーム) をゴールとして送信
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map, stamp: {sec: 0}}, \
    pose: {position: {x: 0.5, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}" --once

# ナビゲーション状態を監視
ros2 topic echo /navigate_to_pose/_action/status --once
```

**合否基準**:
- `/plan` にパスが生成されること
- `/cmd_vel` に速度指令が出力されること
- 目標付近 (xy_goal_tolerance: 0.25m 以内) でナビゲーションが完了すること
- `GoalStatus: SUCCEEDED` を受信すること

**自動化可否**: 実走行が必要 (手動目視確認)  
**優先度**: 高 — システム全体の E2E 確認  
**注意**: 必ず障害物のない広いスペースで実施すること

---

### N8: velocity_smoother パイプライン確認

**目的**: 急激な速度指令が velocity_smoother で平滑化され、collision_monitor を経由して `/cmd_vel` に届くか確認する。

```bash
# トピック存在確認
ros2 topic list | grep -E "cmd_vel"
# 期待: /cmd_vel, /cmd_vel_nav, /cmd_vel_smoothed

# cmd_vel_nav → cmd_vel_smoothed → cmd_vel の遅延確認
ros2 topic hz /cmd_vel_smoothed --window 20
# 期待: ~20 Hz (smoothing_frequency=20.0)

# 急峻なランプ指令を送ってスムージング効果を確認
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
ros2 topic echo /cmd_vel_smoothed --once | grep 'x:'
# 期待: x < 0.3 (平滑化されている)
```

**合否基準**:
- `/cmd_vel_nav` / `/cmd_vel_smoothed` / `/cmd_vel` の 3 トピックが存在すること
- `/cmd_vel_smoothed` レートが 18〜22 Hz の範囲にあること
- max_velocity [0.30, 0.0, 1.0] を超える指令がクランプされること

**自動化可否**: 完全自動  
**優先度**: 中 — 安全な速度制御のパイプライン確認

---

### N9: collision_monitor 緊急停止確認

**目的**: YDLIDAR T-mini Plus がロボット前方の障害物を検出し、collision_monitor が `/cmd_vel` をゼロにするか確認する。

**手順**:
1. ナビゲーション中 (または手動で `/cmd_vel_smoothed` に速度指令を送信)
2. ロボット前方 0.3m 以内に障害物を置く (PolygonStop: 前方 0.35m)
3. `/cmd_vel` がゼロになることを確認

```bash
# 障害物検出前後の /cmd_vel を監視
ros2 topic echo /cmd_vel --field linear.x

# collision_monitor のログ確認
ros2 node info /collision_monitor
```

**合否基準**:
- 障害物が PolygonStop 領域 (`[[0.35, 0.25], [0.35, -0.25], [-0.2, -0.25], [-0.2, 0.25]]`) に入ると `/cmd_vel` が即座に 0 になること
- 障害物除去後に速度指令が再開されること

**自動化可否**: 手動 (物理障害物が必要)  
**優先度**: 高 — 安全機能の確認

---

### N10: 回復行動 (spin / backup) 確認

**目的**: ロボットがスタック状態になったとき、`behavior_server` の `spin` または `backup` 回復行動が自動で実行されるか確認する。

**手順**:
1. ナビゲーション中に目標への経路を物理的にブロックする
2. `progress_checker` のタイムアウト (movement_time_allowance: 10.0s) 後に回復行動が開始されることを確認

```bash
# behavior_server のアクション確認
ros2 action list | grep -E "spin|backup|wait"
# 期待: /spin, /backup, /wait

# スタック時のログ監視
ros2 topic echo /rosout 2>/dev/null | grep -i "recovery\|spin\|backup\|stuck"
```

**合否基準**:
- `/spin` / `/backup` / `/wait` アクションサーバーが存在すること
- スタック後 10〜20 秒以内に回復行動が開始されること (`ros2 topic echo /rosout` でログ確認)

**自動化可否**: 手動 (意図的スタックが必要)  
**優先度**: 中 — 自律走行の堅牢性

---

### N11: waypoint_follower 複数地点訪問

**目的**: `waypoint_follower` が `/follow_waypoints` アクションで複数のウェイポイントを順次訪問するか確認する。

```bash
# アクション確認
ros2 action list | grep waypoint

# 2点のウェイポイントを送信
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{
  poses: [
    {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.5}, orientation: {w: 1.0}}}
  ]
}" --feedback
```

**合否基準**:
- `/follow_waypoints` アクションサーバーが存在すること
- 1点目到達後に 2点目へ向かうこと (feedback で `current_waypoint` が変化)
- 全点到達後に `SUCCEEDED` を返すこと

**自動化可否**: 実走行が必要 (手動目視確認)  
**優先度**: 低 — 高度なナビゲーション機能

---

### N12: lifecycle bond — ノード障害時の lifecycle_manager 対応

**目的**: lifecycle_manager_navigation が管理するノードがクラッシュした場合に lifecycle_manager が検知し、システム全体のリカバリが動作するか確認する。  
`bond_timeout: 30.0s` — ノードが 30 秒応答しない場合に lifecycle_manager がシステムを再起動する。

```bash
# controller_server を強制終了
kill -9 $(ros2 node info /controller_server | grep PID | awk '{print $2}')

# lifecycle_manager のログを監視 (30s 以内に対応するはず)
ros2 topic echo /rosout 2>/dev/null | grep -i "lifecycle\|bond\|restart" | head -10

# 全ノードが再び active になるか確認
sleep 35
for node in controller_server smoother_server planner_server; do
  ros2 lifecycle get /$node 2>/dev/null
done
```

**合否基準**:
- bond タイムアウト (30s) 以内に lifecycle_manager がノード障害を検知すること
- システム全体が再起動し、全ノードが再び `active` になること

**自動化可否**: 完全自動  
**優先度**: 低 — 長時間自律稼働の堅牢性

---

## 優先度マトリクス

| 優先度 | ID | 理由 |
|--------|----|------|
| 高 | N1 | 9ノード起動失敗 → 全 Nav2 機能不動 |
| 高 | N2 | /map・/scan・TF 欠損 → costmap・プランニング不可 |
| 高 | N3 | グローバルコストマップ無効 → パスプランニング不可 |
| 高 | N4 | ローカルコストマップ無効 → 動的障害物回避不可 |
| 高 | N5 | アクションサーバー不在 → ゴール受付不可 |
| 高 | N6 | パス生成失敗 → 到達不可 |
| 高 | N7 | E2E ナビ失敗 → 自律移動不可 |
| 高 | N9 | 緊急停止失敗 → 衝突事故リスク |
| 中 | N8 | velocity_smoother 障害 → 急峻な速度変化 |
| 中 | N10 | 回復行動なし → スタック時に人間介入が必要 |
| 低 | N11 | waypoint_follower → 高度な自律走行 |
| 低 | N12 | bond タイムアウト → 長時間稼働の堅牢性 |

---

## システム構成サマリー

```
[slam_toolbox]
  /scan (YDLIDAR 10Hz) → /map (3s更新) + TF map→odom

[Nav2 lifecycle_manager_navigation]
  planner_server   : /map + /odom + goal → /plan (A* グローバルパス)
  controller_server: /plan + /odom + /tf → /cmd_vel_nav (10Hz, RegPurePursuit)
  velocity_smoother: /cmd_vel_nav → /cmd_vel_smoothed (20Hz, 平滑化)
  collision_monitor: /cmd_vel_smoothed + /scan → /cmd_vel (緊急停止)
  behavior_server  : スタック時 spin/backup/wait
  bt_navigator     : /goal_pose → NavigateToPose BT
  smoother_server  : パス平滑化
  route_server     : ルートグラフナビ (Jazzy)
  waypoint_follower: 複数地点順次訪問

[collision_monitor パラメータ]
  PolygonStop: [[0.35, 0.25], [0.35,-0.25], [-0.2,-0.25], [-0.2, 0.25]]
  障害物 max_points: 3 点以上で即時停止

[velocity_smoother パラメータ]
  max_velocity: [0.30 m/s, 0, 1.0 rad/s]
  max_accel:    [0.5 m/s², 0, 1.0 rad/s²]
```

---

## 全体進捗

| ID | 内容 | 状態 |
|----|------|------|
| N1 | 9ノード起動 & lifecycle active | **確認済み** (2026-04-05) |
| N2 | 必須トピック・TF チェーン | 未実施 |
| N3 | グローバルコストマップ | 未実施 |
| N4 | ローカルコストマップ | 未実施 |
| N5 | navigate_to_pose アクションサーバー | 未実施 |
| N6 | グローバルパスプランニング | 未実施 |
| N7 | ナビゲーション E2E | 未実施 |
| N8 | velocity_smoother パイプライン | 未実施 |
| N9 | collision_monitor 緊急停止 | 未実施 |
| N10 | 回復行動 (spin / backup) | 未実施 |
| N11 | waypoint_follower 複数地点訪問 | 未実施 |
| N12 | lifecycle bond ノード障害対応 | 未実施 |
