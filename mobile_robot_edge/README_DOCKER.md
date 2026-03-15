# Mobile Robot Edge - Docker環境での使用方法

## 概要

このパッケージは、Raspberry Pi上のDocker環境でROS 2 Jazzyを使用し、edge_servicesとWebSocket経由で通信します。

## アーキテクチャ

```
┌─────────────────────────────────────────┐
│ Docker Container (ROS 2 Jazzy)         │
│                                         │
│  ┌───────────────────────────────────┐ │
│  │ mobile_robot_edge package         │ │
│  │                                   │ │
│  │ - ws_motor_controller             │ │
│  │   subscribes: /cmd_vel            │ │
│  │   → WebSocket → motor_service     │ │
│  │                                   │ │
│  │ - ws_odometry_publisher           │ │
│  │   ← WebSocket ← odometry_service  │ │
│  │   publishes: /odom, TF            │ │
│  │                                   │ │
│  │ - robot_state_publisher           │ │
│  │   publishes: TF (static)          │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
        ↕ WebSocket (localhost:8001-8003)
┌─────────────────────────────────────────┐
│ Host (Raspberry Pi - Debian 13)        │
│                                         │
│  ┌───────────────────────────────────┐ │
│  │ edge_services (Python)            │ │
│  │                                   │ │
│  │ - motor_service.py                │ │
│  │   WebSocket server: 8001          │ │
│  │                                   │ │
│  │ - odometry_service.py             │ │
│  │   WebSocket server: 8002          │ │
│  │                                   │ │
│  │ - camera_service.py               │ │
│  │   WebSocket server: 8003          │ │
│  └───────────────────────────────────┘ │
│           ↓ I2C (/dev/i2c-1)           │
│  ┌───────────────────────────────────┐ │
│  │ PR2040 Motor Driver (0x60)        │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

## セットアップ

### 1. edge_servicesの起動（ホスト側）

別のターミナルでedge_servicesを起動:

```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

これにより以下のサービスが起動します:
- モーター制御: `ws://localhost:8001`
- オドメトリ: `ws://localhost:8002`
- カメラ: `ws://localhost:8003`

### 2. Docker環境でROS 2を起動

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2
```

### 3. パッケージのビルド（初回のみ）

コンテナ内で:

```bash
cd /ros2_ws
colcon build --packages-select mobile_robot_edge
source install/setup.bash
```

### 4. ノードの起動

#### すべてのノードを起動（推奨）

```bash
ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
```

起動されるノード:
- `ws_motor_controller` - /cmd_velを受信してモーター制御
- `ws_odometry_publisher` - オドメトリデータを配信
- `robot_state_publisher` - ロボットのTFツリーを配信

#### 個別ノードの起動

```bash
# モーターコントローラのみ
ros2 run mobile_robot_edge ws_motor_controller

# オドメトリパブリッシャのみ
ros2 run mobile_robot_edge ws_odometry_publisher
```

## 動作確認

### トピックの確認

別のターミナルでDockerコンテナに入り:

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2
```

コンテナ内で:

```bash
# トピックリストを表示
ros2 topic list

# オドメトリデータを表示
ros2 topic echo /odom

# TFツリーを確認
ros2 run tf2_tools view_frames
```

### モーター制御のテスト

コンテナ内で速度指令を送信:

```bash
# 前進 (0.2 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 回転 (0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## パラメータ設定

### Launch引数

```bash
ros2 launch mobile_robot_edge ws_edge_bringup.launch.py \
  motor_ws_uri:=ws://localhost:8001 \
  odom_ws_uri:=ws://localhost:8002 \
  cmd_vel_topic:=/cmd_vel
```

### ノードパラメータ

#### ws_motor_controller

- `ws_uri`: モーターサービスのWebSocket URI（デフォルト: ws://localhost:8001）
- `cmd_vel_topic`: 速度指令トピック名（デフォルト: /cmd_vel）

#### ws_odometry_publisher

- `ws_uri`: オドメトリサービスのWebSocket URI（デフォルト: ws://localhost:8002）
- `frame_id`: 親フレームID（デフォルト: odom）
- `child_frame_id`: 子フレームID（デフォルト: base_footprint）

## ノード仕様

### ws_motor_controller

**購読トピック:**
- `/cmd_vel` (geometry_msgs/Twist) - 速度指令

**WebSocket送信:**
```json
{
  "type": "cmd_vel",
  "linear": 0.2,
  "angular": 0.5
}
```

**機能:**
- ROS 2の/cmd_velを受信
- WebSocket経由でmotor_serviceへ転送
- 接続が切れた場合は自動再接続

### ws_odometry_publisher

**WebSocket受信:**
```json
{
  "type": "odom",
  "timestamp": 1234567.89,
  "pose": {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0
  },
  "twist": {
    "linear": 0.0,
    "angular": 0.0
  },
  "encoders": [0, 0, 0, 0]
}
```

**配信トピック:**
- `/odom` (nav_msgs/Odometry) - オドメトリ情報

**配信TF:**
- `odom` → `base_footprint`

**機能:**
- odometry_serviceからWebSocket経由でデータ受信
- ROS 2のOdometryメッセージとTFに変換して配信
- 接続が切れた場合は自動再接続

## トラブルシューティング

### WebSocketに接続できない

1. edge_servicesが起動しているか確認:
```bash
# ホスト側で
ps aux | grep python3
netstat -tuln | grep 800
```

2. edge_servicesのログを確認:
```bash
tail -f /home/pi/MobileRobot/edge_services/logs/edge_services.log
```

### ノードが起動しない

1. パッケージが正しくビルドされているか確認:
```bash
# Docker内で
cd /ros2_ws
colcon build --packages-select mobile_robot_edge
source install/setup.bash
```

2. 依存関係が満たされているか確認:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### オドメトリデータが配信されない

1. odometry_serviceが動作しているか確認
2. WebSocket接続ログを確認:
```bash
ros2 run mobile_robot_edge ws_odometry_publisher --ros-args --log-level debug
```

## 次のステップ

1. **Navigation Stackの統合**
   - Nav2パッケージをDockerイメージに追加
   - ナビゲーションパラメータの設定

2. **SLAMの実装**
   - SLAM Toolboxの統合
   - カメラデータの活用

3. **自律ナビゲーション**
   - ゴール設定機能の実装
   - 障害物回避の実装

## 関連ドキュメント

- [edge_services README](../edge_services/README.md)
- [Docker Setup README](../ros2_docker/README.md)
- [Docker Quickstart](../ros2_docker/QUICKSTART.md)
