# LiDAR SLAM 検証項目

RTAB-Map 3D LiDAR ICP SLAM の検証手順書。
`ros2_docker/VERIFICATION.md` の R1〜R10 が PASS であることを前提とする。

---

## 検証前の準備

### ビルド

```bash
# イメージ再ビルド (pyserial 追加のため必要)
cd ~/MobileRobot/ros2_docker
sudo docker compose build --no-cache

# パッケージビルド
sudo docker compose run --rm ros2 bash -c "
  cd /ros2_ws &&
  colcon build --packages-select mobile_robot_server mobile_robot_edge &&
  echo 'BUILD OK'"
```

### udev ルール設定（推奨）

接続されている ESP32 の VID/PID を確認し、デバイスパスを固定する:

```bash
# ホスト上で実行 (Docker 外)
lsusb | grep -i esp            # VID:PID を確認
ls /dev/ttyACM*                # ポート番号を確認

# /etc/udev/rules.d/99-mobile-robot.rules に追記
# SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", SYMLINK+="ttyLidar"
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 起動順序

```bash
# ターミナル A: edge_services
cd ~/MobileRobot/edge_services && python3 launch_all.py

# ターミナル B: ROS 2 ベース
sudo docker compose run --rm ros2 bash -c "
  source /ros2_ws/install/setup.bash &&
  ros2 launch mobile_robot_edge ws_edge_bringup.launch.py"

# ターミナル C: SLAM
sudo docker compose run --rm ros2 bash -c "
  source /ros2_ws/install/setup.bash &&
  ros2 launch mobile_robot_server slam_bringup.launch.py"
```

---

## 現状の検証カバレッジ

| ID | 内容 | 状態 |
|----|------|------|
| SLAM_S1 | lidar_pointcloud_bridge 起動確認 | 未検証 (実機必要) |
| SLAM_S2 | PointCloud2 品質確認 | 未検証 (実機必要) |
| SLAM_S3 | TF ツリー完全性確認 | 未検証 (実機必要) |
| SLAM_S4 | RTAB-Map 起動確認 | 未検証 (実機必要) |
| SLAM_S5 | 静止時の自己位置安定性 | 未検証 (実機必要) |
| SLAM_S6 | 直進 1 m の地図整合性 | 未検証 (実機必要) |
| SLAM_S7 | ループクロージャ確認 | 未検証 (実機必要) |
| SLAM_S8 | 地図保存・ロード | 未検証 (実機必要) |
| SLAM_S9 | CPU・メモリ使用量 | 未検証 (実機必要) |

## ソフトウェアレベル検証結果 (実機不要)

| テスト | 内容 | 結果 |
|--------|------|------|
| T1–T6  | バイナリプロトコル (フレームビルド・パース・チェックサム) | PASS 31/31 |
| T7–T9  | パーサー堅牢性 (断片化・ゴミデータ・バースト) | PASS 13/13 |
| T10a–b | IMU 回転行列定義 (Ry/Rx) | PASS |
| T10c   | IMU 補正演算順序 SPEC §5.3 と一致 | PASS |
| T10d   | np.stack 形状と変換後形状 | PASS |
| T10e   | ピッチ 5° ラウンドトリップ検証 | PASS |
| T10f   | ロール 5° ラウンドトリップ検証 | PASS |
| T10h   | PointCloud2 バイナリレイアウト (field offset 0/4/8/12) | PASS |
| frame_id | `lidar_base_link` が cloud_frame、`lidar_tilt_link` が TF 可視化専用 | PASS (コード検証済) |
| T11    | CONTINUOUS モード STATUS(2) 送信バグ → **ファームウェア修正済** | FIXED |
| T12    | CMD_SCAN_START ペイロード (13バイト)・launch パラメータ型変換 | PASS |
| T13    | IMU データ流入時のスキャンウォッチドッグ停止バグ → **ブリッジ修正済** | FIXED |
| T14    | フル統合: プロトコル + スキャンサイクル + PointCloud2 出力 | PASS |
| T15–T16 | rpi_tilt_3d.py CONTINUOUS モード修正・SPEC status シーケンス整合 | PASS 6/6 |
| T17    | stopScan 途中停止: 部分クラウド発行・空スキャン無発行 | PASS |
| T19    | RTAB-Map パラメータ名・値・モード整合性 (42 params) | PASS |
| T20    | スキャンレート 0.32 Hz、最大安全速度 0.16 m/s 算出 | PASS |
| T21    | static_transform_publisher 引数数・全回転ゼロ確認 | PASS |
| T22    | CMD_SCAN_STOP フレーム・ファームウェアパーサー整合 | PASS |
| T23    | destroy_node() グレースフルシャットダウン安全性 | PASS |
| T24    | ファームウェア rxFrame[128] バッファ境界確認 | PASS |
| T25    | _total_frames カウンタ 24h 稼働オーバーフローなし | PASS |
| T26–T31 | launch/setup.py/Dockerfile/compose/bridge/firmware 最終確認 | PASS 24/25* |

\* T30 の 1 FAIL は文字列照合パターンのミス（ソース正常）。`declare_parameter('tilt_frame', 'lidar_tilt_link')` は行 188 で確認済み。

## 発見・修正済みバグ

### BUG-1: PointCloud2 の frame_id が lidar_tilt_link (二重チルト回転)
- **症状**: RTAB-Map がチルト回転を二重適用 → 点群が歪む
- **原因**: `lidar_frame` パラメータが `lidar_tilt_link` にデフォルト設定されていたが、
  `_slice_to_points` 内部で既に Ry(alpha) を適用済み
- **修正**: `cloud_frame='lidar_base_link'`（PointCloud2）と `tilt_frame='lidar_tilt_link'`（TF可視化）に分離
- **ファイル**: `lidar_pointcloud_bridge.py`

### BUG-2: CONTINUOUS モードで PointCloud2 が発行されない
- **症状**: `slam_bringup.launch.py` 起動時 (`SCAN_MODE_CONTINUOUS`)、`/lidar/points` が一度も発行されない
- **原因**: ファームウェアの `S3D_COMPLETE` (連続モード) で `sendStatus(2, ...)` が呼ばれておらず、
  ブリッジの `_publish_cloud()` がトリガーされない
- **修正**: `S3D_COMPLETE` 連続モードに `sendStatus(2, totalSteps, totalSteps)` および `sendStatus(0, 0, totalSteps)` を追加
- **ファイル**: `LiderModule/firmware/lidar_tilt_3d/lidar_tilt_3d.ino`

### BUG-3: IMU パケット流入時にスキャン再送ウォッチドッグが停止
- **症状**: ブリッジ接続直後、ESP32 が IMU データ (TYPE 0x02, 50Hz) を送信するため
  `last_rx` が更新され続け、30秒ウォッチドッグが発火しない。
  LiDAR ウォームアップ前に CMD_SCAN_START を送ると無言で拒否されるが、
  スキャンが永遠に開始されない。
- **原因**: `_read_loop` の `last_rx` はあらゆるバイトで更新されるが、
  スキャンフレーム (SCAN_SLICE/SCAN_STATUS) の受信とは区別されていない
- **修正**: `last_scan_rx` を追加し、10秒以内にスキャンフレームが届かない場合に
  CMD_SCAN_START を再送する
- **ファイル**: `lidar_pointcloud_bridge.py` `_read_loop`

### BUG-4: RTAB-Map frame_id が base_link (2D グリッド基準が床面からずれる)
- **症状**: 占有グリッドの Z 基準が `base_link` (ロボット胴体) になるため、
  `Grid/FootprintHeight: "0.1"` が床面ではなくロボット胴体レベルを基準に適用される。
  床点が占有グリッドに混入するリスクがある。
- **原因**: `rtabmap_lidar_3d_params.yaml` の `frame_id` に `base_link` を設定。
  カメラ版 `rtabmap_params.yaml` は `base_footprint` (床レベル) を使用している。
- **修正**: `frame_id: base_footprint` に変更。TF チェーン
  `lidar_base_link → base_link → base_footprint` は TF2 で到達可能。
- **ファイル**: `mobile_robot_server/config/rtabmap_lidar_3d_params.yaml`

---

## 検証項目 詳細

---

### SLAM_S1: lidar_pointcloud_bridge 起動確認

**目的**: ブリッジノードが ESP32 に接続し、スキャンコマンドを送信してデータ受信を開始するか確認する。

**手順**:
```bash
# ノード起動確認
ros2 node list | grep lidar_pointcloud_bridge

# 起動ログ確認 (別ターミナル)
ros2 node info /lidar_pointcloud_bridge
```

**期待されるログ出力**:
```
[lidar_pointcloud_bridge] LiDAR bridge — port=/dev/ttyACM1 tilt=[-30.0°,0.0°] step=3.0°
[lidar_pointcloud_bridge] Opening /dev/ttyACM1 …
[lidar_pointcloud_bridge] Connected — sending scan start
```

**合否基準**:
- `/lidar_pointcloud_bridge` がノードリストに現れること
- "Connected" のログが出力されること
- "Serial error" や "retrying" が繰り返し出ないこと

**トラブルシューティング**:
- `Permission denied`: `sudo chmod 666 /dev/ttyACM1` またはユーザーを `dialout` グループに追加
- `No such file`: `ls /dev/ttyACM*` でポート番号を確認し、`lidar_port` 引数を変更

**優先度**: 高

---

### SLAM_S2: PointCloud2 品質確認

**目的**: `/lidar/points` に有効な 3D ポイントが正しいフレームで発行されるか確認する。

**手順**:
```bash
# [1] トピック発行確認
ros2 topic list | grep lidar

# [2] レート確認 (~0.3 Hz)
ros2 topic hz /lidar/points --window 10

# [3] メッセージ内容確認
ros2 topic echo /lidar/points --once | head -30
# frame_id: lidar_base_link, width: 3000以上 が期待値

# [4] スキャン状態確認
ros2 topic echo /lidar/scan_status
# 'scanning' → 'idle' → 'scanning' ... の繰り返し

# [5] ポイント数確認 (Python ワンライナー)
ros2 run rclpy_tools topic_print.py /lidar/points \
  --expression 'f"pts={msg.width} frame={msg.header.frame_id}"' --once
```

**合否基準**:
- `/lidar/points` が ≥ 0.2 Hz で発行されること
- `frame_id` が `lidar_base_link` であること（チルト回転は内部で適用済みのため）
- `width` ≥ 3,000 pts (スライス数 × 667 pts/スライス × フィルタ後)

**優先度**: 高

---

### SLAM_S3: TF ツリー完全性確認

**目的**: TF チェーンが切れていないか、および PointCloud2 フレーム (`lidar_base_link`) が正しいか確認する。

**フレーム設計メモ**:
- `/lidar/points` の `frame_id` = `lidar_base_link`（チルト回転は `_slice_to_points` 内部で適用済み）
- `lidar_tilt_link` はリアルタイム可視化専用（RTAB-Map は参照しない）
- RTAB-Map は `lidar_base_link` フレームの PointCloud2 を `scan_cloud` として受け取る

**手順**:
```bash
# [1] TF ツリー全体を出力
ros2 run tf2_tools view_frames
# map (from rtabmap) → odom → base_footprint → base_link → lidar_base_link → lidar_tilt_link
# が含まれることを確認

# [2] 各フレーム間の変換確認
ros2 run tf2_ros tf2_echo base_link lidar_base_link
# 期待: translation z≈0.12, rotation all zero

ros2 run tf2_ros tf2_echo lidar_base_link lidar_tilt_link
# 期待: translation all zero, rotation.y が tilt_deg に応じて変化

ros2 run tf2_ros tf2_echo map lidar_tilt_link
# チェーン全体が繋がっていれば変換値が返る

# [3] TF 更新レート確認
ros2 topic hz /tf --window 50
# lidar_tilt_link は 1 スライスごと (~3.5 Hz) に更新される
```

**合否基準**:
- `map → lidar_base_link` のフルチェーンが存在すること
- `lidar_tilt_link` の rotation.y が ±0.5 rad の範囲で変動すること (チルト動作中)
- `tf2_echo` がタイムアウトしないこと

**優先度**: 高

---

### SLAM_S4: RTAB-Map 起動確認

**目的**: RTAB-Map が ICP モードで起動し、地図トピックを発行するか確認する。

**手順**:
```bash
# [1] ノード起動確認
ros2 node list | grep rtabmap

# [2] トピック発行確認
ros2 topic list | grep -E "^/map$|cloud_map|localization_pose"
# 期待: /map, /rtabmap/cloud_map, /rtabmap/localization_pose

# [3] map トピック確認
ros2 topic echo /map --once | head -10
# frame_id: map, resolution: 0.05 が期待値

# [4] ログ確認 (エラーなし)
ros2 node info /rtabmap
# [RTAB-Map] が INFO レベルで起動ログを出力していること
```

**合否基準**:
- `/rtabmap` ノードが起動していること
- `/map` (OccupancyGrid) が発行されること
- `/rtabmap/cloud_map` (PointCloud2) が発行されること
- ERROR ログが繰り返し出ないこと

**優先度**: 高

---

### SLAM_S5: 静止時の自己位置安定性

**目的**: ロボットを静止させた状態で、推定位置が安定しているか確認する。

**手順**:
```bash
# [1] 初期位置を記録
ros2 topic echo /rtabmap/localization_pose --once

# [2] 30 秒待機（ロボットは静止）

# [3] 位置を再確認
ros2 topic echo /rtabmap/localization_pose --once

# [4] /odom と位置ずれを確認
ros2 run tf2_ros tf2_echo map odom
# translation の変化が小さいことを確認
```

**合否基準**:
- 30 秒間の位置変化 < 1 cm (x, y)
- 角度変化 < 0.05 rad

**優先度**: 中

---

### SLAM_S6: 直進 1 m の地図整合性

**目的**: ロボットを 1 m 直進させたときに、地図上の軌跡が直線になるか確認する。

**手順**:
```bash
# [1] /odom で初期位置を確認
ros2 topic echo /odom --once | grep position

# [2] ロボットを前進させる
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --times 100 --rate 20
# 5秒間 × 0.1 m/s = 0.5 m 前進
# 2回実行で 1 m

# [3] /odom で到達位置を確認
ros2 topic echo /odom --once | grep position

# [4] RViz でポイントクラウド軌跡を目視確認
# /rtabmap/cloud_map トピックを表示 → 壁・床が直線であること
```

**合否基準**:
- `/odom` の x 変化量が ≈ 1.0 m (±10%)
- RViz の cloud_map で壁が直線・床が平面に見えること

**優先度**: 中

---

### SLAM_S7: ループクロージャ確認

**目的**: 過去に通過した場所に戻ったとき、RTAB-Map がループクロージャを検出して TF を修正するか確認する。

**手順**:
```bash
# [1] ロボットを約 2m × 2m の正方形に沿って走行させる
# (手動操縦でスタート地点に戻る)

# [2] RTAB-Map のループ検出ログを確認
ros2 topic echo /rtabmap/info --once
# loop_closure_id > 0 になればループクロージャ検出

# [3] TF の修正を確認
ros2 run tf2_ros tf2_echo map odom
# ループ検出前後で translation が急に修正されることを確認
```

**合否基準**:
- ループクロージャが ≥ 1 回検出されること (`/rtabmap/info` の `loop_closure_id > 0`)
- マップが閉じた形状になること (RViz 目視)

**優先度**: 中

---

### SLAM_S8: 地図保存・ロード

**目的**: SLAM で作成したマップを保存し、再起動後にロケーションモードで使用できるか確認する。

**手順**:

#### 保存
```bash
# 地図保存 (SLAM 実行中に実行)
ros2 service call /rtabmap/save_map std_srvs/srv/Empty
# /maps/rtabmap.db に保存される

# 保存確認
ls -lh /maps/rtabmap.db
```

#### ロード・ロケーション
```bash
# SLAM ノードを停止してからロケーションモードで再起動
ros2 launch mobile_robot_server slam_bringup.launch.py localization:=true

# 地図が表示されることを確認
ros2 topic echo /map --once
# 保存した地図が /map として発行されること
```

**合否基準**:
- `/maps/rtabmap.db` が作成されること
- ロケーションモード起動後に `/map` が発行されること
- `/rtabmap/localization_pose` が既知の環境で収束すること

**優先度**: 低

---

### SLAM_S9: CPU・メモリ使用量

**目的**: Raspberry Pi 4 上での CPU・メモリ使用量が許容範囲内か確認する。

**手順**:
```bash
# Docker コンテナ外から監視 (ホスト上)
watch -n 2 "docker stats mobile_robot_ros2 --no-stream"

# コンテナ内での詳細確認
htop   # CPU コア別使用率、メモリ使用量

# ROS 2 ノード別 CPU 使用率
top -p $(ps aux | grep rtabmap | awk 'NR==1{print $2}')
```

**合否基準**:
- 全体 CPU 使用率 < 80%
- メモリ使用量 < 2 GB
- RTAB-Map ノード単体 CPU < 50%
- システムが OOM Killer に落とされないこと

**優先度**: 低

---

## 優先度マトリクス

| 優先度 | ID | 理由 |
|--------|----|------|
| 高 | SLAM_S1 | ブリッジ起動なし → SLAM 不動 |
| 高 | SLAM_S2 | PointCloud2 品質不良 → ICP 収束不可 |
| 高 | SLAM_S3 | TF 欠損 → RTAB-Map 動作不可 |
| 高 | SLAM_S4 | RTAB-Map 未起動 → 地図なし |
| 中 | SLAM_S5 | 位置ドリフト → ナビゲーション精度低下 |
| 中 | SLAM_S6 | 地図歪み → 経路計画不正 |
| 中 | SLAM_S7 | ループクロージャなし → 長時間走行でドリフト累積 |
| 低 | SLAM_S8 | 地図保存なし → 毎回 SLAM をやり直し |
| 低 | SLAM_S9 | リソース超過 → クラッシュリスク |

---

## チューニングガイド

### SLAM 中のロボット最大速度

デフォルト設定 (`tilt_min=-30°, tilt_max=0°, step=3°`) での制約:

| 項目 | 値 |
|------|-----|
| スキャン周期 | ~3.2 秒 (11 ステップ × 287 ms) |
| ICP/MaxTranslation | 0.5 m |
| **最大安全速度** | **~0.16 m/s** (MaxTranslation ÷ scan_period) |

この速度を超えると ICP のスキャン間重複が減少し、ループクロージャ失敗のリスクが高まる。

速度を上げたい場合は `tilt_step` を大きくしてスキャン周期を短縮:
```
tilt_step=5.0 → 7 steps × 287ms ≈ 2.0s → 最大速度 0.25 m/s
tilt_step=10.0 → 4 steps × 287ms ≈ 1.1s → 最大速度 0.45 m/s (点密度は低下)
```

### PointCloud が少ない / 空の場合

```bash
# min_quality を下げる (デフォルト 10 → 5)
ros2 launch mobile_robot_server slam_bringup.launch.py \
  lidar_port:=/dev/ttyACM1 \
  # → lidar_pointcloud_bridge のパラメータを直接変更
```

`slam_bringup.launch.py` の `lidar_bridge` ノードのパラメータに追加:
```python
'min_quality': 5,
'min_dist_mm': 20,
```

### ICP が収束しない場合

`rtabmap_lidar_3d_params.yaml` を調整:

```yaml
Icp/MaxCorrespondenceDistance: "1.0"   # 0.5 → 1.0 に緩める
Icp/Iterations: "50"                    # 30 → 50 に増やす
RGBD/LinearUpdate: "0.05"              # 0.10 → 0.05 に小さくする
```

### CPU 使用率が高い場合

```yaml
# ボクセルサイズを大きくしてポイント数を削減
Icp/VoxelSize: "0.10"    # 0.05 → 0.10

# ループ検出レートを下げる
Rtabmap/DetectionRate: "0.5"   # 1 → 0.5 Hz

# tilt_step を大きくしてスキャン時間を短縮
# slam_bringup.launch.py: tilt_step:=5.0 (3.0 → 5.0)
```

### スキャンタイムアウトが頻発する場合

ファームウェア側の問題の可能性:
1. `rpi_tilt_3d.py` でスキャンが正常に動作するか確認
2. ESP32 のシリアルポートを確認 (`ls -la /dev/ttyACM*`)
3. `lidar_port` パラメータが正しいか確認

### RTAB-Map が "TF lookup failed" を出力する場合

`subscribe_odom: false` (TF 経由) を使用しているため、TF チェーンが必要:

```bash
# odom → base_link の TF が通っているか確認
ros2 run tf2_ros tf2_echo odom base_link

# ws_edge_bringup が起動しているか確認
ros2 node list | grep ws_odometry_publisher
```

`ws_edge_bringup.launch.py` が起動していない場合は先に起動してから SLAM を開始すること。
