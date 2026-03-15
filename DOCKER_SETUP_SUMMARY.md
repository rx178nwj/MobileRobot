# Mobile Robot - Docker ROS 2 セットアップ完了

## 完了した作業

### 1. Docker環境構築
- ✅ Docker 29.3.0 インストール
- ✅ ROS 2 Jazzy Dockerfileの作成
- ✅ docker-compose設定の作成
- ✅ Dockerイメージのビルド (mobile-robot-ros2:jazzy)
- ✅ ハードウェアアクセス設定 (I2C, カメラ)

### 2. mobile_robot_edgeパッケージの更新
- ✅ WebSocketブリッジノードの作成
  - `ws_motor_controller.py` - /cmd_vel → motor_service
  - `ws_odometry_publisher.py` - odometry_service → /odom
- ✅ setup.pyの更新（新しいエントリポイント追加）
- ✅ package.xmlの更新（robot_state_publisher依存追加）
- ✅ ws_edge_bringup.launch.pyの作成
- ✅ Docker環境でのビルド確認

## ディレクトリ構成

```
/home/pi/MobileRobot/
├── ros2_docker/
│   ├── Dockerfile                    # ROS 2 Jazzy定義
│   ├── docker-compose.yml            # コンテナ設定
│   ├── README.md                     # 詳細ドキュメント
│   ├── QUICKSTART.md                 # クイックリファレンス
│   └── run_edge_bringup.sh          # 起動スクリプト
│
├── mobile_robot_edge/
│   ├── mobile_robot_edge/
│   │   ├── ws_motor_controller.py    # WebSocketモーター制御
│   │   ├── ws_odometry_publisher.py  # WebSocketオドメトリ配信
│   │   ├── odometry_publisher.py     # I2C直接通信版（保持）
│   │   └── motor_controller.py       # I2C直接通信版（保持）
│   ├── launch/
│   │   ├── ws_edge_bringup.launch.py # WebSocket版起動ファイル
│   │   ├── edge_bringup.launch.py    # I2C版起動ファイル
│   │   └── camera.launch.py          # カメラ起動ファイル
│   ├── package.xml
│   ├── setup.py
│   └── README_DOCKER.md              # Docker環境用README
│
├── edge_services/
│   ├── motor_service.py              # WebSocket:8001
│   ├── odometry_service.py           # WebSocket:8002
│   ├── camera_service.py             # WebSocket:8003
│   ├── launch_all.py
│   └── README.md
│
└── DOCKER_SETUP_SUMMARY.md           # 本ファイル
```

## 使用方法

### クイックスタート

#### 1. edge_servicesの起動（ターミナル1）

```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

#### 2. ROS 2ノードの起動（ターミナル2）

```bash
cd /home/pi/MobileRobot/ros2_docker
./run_edge_bringup.sh
```

または手動で:

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2 bash -c "source /ros2_ws/install/setup.bash && ros2 launch mobile_robot_edge ws_edge_bringup.launch.py"
```

#### 3. 動作確認（ターミナル3）

```bash
# Dockerコンテナに入る
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2

# コンテナ内で
source /ros2_ws/install/setup.bash

# トピック確認
ros2 topic list
ros2 topic echo /odom

# モーター制御テスト
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## システムアーキテクチャ

```
┌─────────────────────────────────────────┐
│ Docker Container (Ubuntu 24.04)        │
│                                         │
│  ┌───────────────────────────────────┐ │
│  │ ROS 2 Jazzy                       │ │
│  │                                   │ │
│  │ - ws_motor_controller             │ │
│  │   /cmd_vel → WS:8001              │ │
│  │                                   │ │
│  │ - ws_odometry_publisher           │ │
│  │   WS:8002 → /odom, TF             │ │
│  │                                   │ │
│  │ - robot_state_publisher           │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
        ↕ WebSocket (network_mode: host)
┌─────────────────────────────────────────┐
│ Raspberry Pi (Debian 13)               │
│                                         │
│  ┌───────────────────────────────────┐ │
│  │ edge_services (Python)            │ │
│  │ - motor_service (WS:8001)         │ │
│  │ - odometry_service (WS:8002)      │ │
│  │ - camera_service (WS:8003)        │ │
│  └───────────────────────────────────┘ │
│           ↓ I2C, Camera               │
│  ┌───────────────────────────────────┐ │
│  │ Hardware                          │ │
│  │ - PR2040 (I2C 0x60)               │ │
│  │ - Camera Module 3                 │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

## 通信フロー

### モーター制御
```
外部PC/Nav2 → /cmd_vel (ROS 2) → ws_motor_controller
  → WebSocket (8001) → motor_service
  → I2C → PR2040 → モーター
```

### オドメトリ
```
エンコーダ → PR2040 → I2C → odometry_service
  → WebSocket (8002) → ws_odometry_publisher
  → /odom (ROS 2) → 外部PC/Nav2
```

## ノード仕様

### ws_motor_controller
- **パッケージ**: mobile_robot_edge
- **購読**: /cmd_vel (geometry_msgs/Twist)
- **WebSocket**: ws://localhost:8001
- **機能**:
  - ROS 2速度指令をWebSocket経由でmotor_serviceへ転送
  - 自動再接続機能

### ws_odometry_publisher
- **パッケージ**: mobile_robot_edge
- **配信**: /odom (nav_msgs/Odometry)
- **TF**: odom → base_footprint
- **WebSocket**: ws://localhost:8002
- **機能**:
  - odometry_serviceからWebSocket経由でデータ受信
  - ROS 2 OdometryとTFに変換して配信
  - 自動再接続機能

### robot_state_publisher
- **パッケージ**: robot_state_publisher
- **機能**: ロボットの静的TFツリーを配信

## パラメータ

### ws_edge_bringup.launch.py

```bash
ros2 launch mobile_robot_edge ws_edge_bringup.launch.py \
  motor_ws_uri:=ws://localhost:8001 \
  odom_ws_uri:=ws://localhost:8002 \
  cmd_vel_topic:=/cmd_vel
```

## トラブルシューティング

### 1. WebSocketに接続できない

**確認項目:**
```bash
# edge_servicesが起動しているか
ps aux | grep python3

# ポートが開いているか
netstat -tuln | grep 800

# edge_servicesログ確認
tail -f /home/pi/MobileRobot/edge_services/logs/edge_services.log
```

### 2. I2Cデバイスが見えない

```bash
# ホスト側でI2C確認
i2cdetect -y 1

# 権限確認
ls -l /dev/i2c-1

# 必要に応じて
sudo chmod 666 /dev/i2c-1
```

### 3. パッケージビルドエラー

```bash
# Docker内で再ビルド
cd /ros2_ws
rm -rf build install log
colcon build --packages-select mobile_robot_edge
```

## 次のステップ

### 1. Navigation Stack統合

```dockerfile
# Dockerfileに追加
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*
```

### 2. SLAM統合

```dockerfile
# Dockerfileに追加
RUN apt-get update && apt-get install -y \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*
```

### 3. カメラ統合

カメラサービス(WS:8003)のROS 2ブリッジノードを作成

### 4. IMU統合

PR2040からIMUデータ(MPU-6881)を取得してROS 2で配信

## 関連ドキュメント

- [Docker Setup README](ros2_docker/README.md) - Docker環境詳細
- [Docker Quickstart](ros2_docker/QUICKSTART.md) - クイックリファレンス
- [Edge Package Docker README](mobile_robot_edge/README_DOCKER.md) - パッケージ使用方法
- [Edge Services README](edge_services/README.md) - Edge Servicesドキュメント

## 既知の制限事項

1. **Debian 13のネイティブROS 2非対応**
   - 依存関係の問題によりDocker環境が必須
   - パフォーマンス: ネイティブ実行と比較して若干のオーバーヘッド

2. **WebSocket通信のレイテンシ**
   - I2C直接通信と比較してわずかな遅延あり
   - リアルタイム制御には影響なし（実測で問題なし）

3. **カメラ統合未完了**
   - camera_serviceは実装済み
   - ROS 2ブリッジノードは未実装（今後の課題）

## 作成者

rx178nwj

## ライセンス

MIT
