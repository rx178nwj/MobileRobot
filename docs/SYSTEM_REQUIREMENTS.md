# モバイルロボット システム要件定義書

> 更新: 2026-04 — LiderModule (YDLIDAR T-mini Pro + STS3215 チルト) 統合後の構成を反映

---

## 1. システム概要

差動二輪モバイルロボットの自律走行システム。Raspberry Pi (エッジ) と Ubuntu デスクトップ (サーバー) の2ノード構成で、LiDAR SLAM・Nav2・LLM ナビゲーションを実現する。

### 1.1 構成図

```
┌─────────────────────────────────────────────────────────────────┐
│  Ubuntu Server (デスクトップ PC)                                  │
│                                                                   │
│  slam_toolbox ─── /map ──────────────────────────────────────── │
│  (LiDAR SLAM)     /tf(map→odom)                                  │
│       ↑                                                           │
│     /scan ◄── [Zenoh / WiFi] ◄─────────────────────────────┐    │
│                                                             │    │
│  Nav2 ─── /cmd_vel ─── [Zenoh / WiFi] ───────────────────┐ │    │
│                                                           │ │    │
│  llm_nav_controller (YOLO yolov8s + Ollama qwen3.5:9b)   │ │    │
│       ↑ /camera/image_raw                                │ │    │
└───────────────────────────────────────────────────────────┼─┼────┘
            Zenoh / WiFi                                    │ │
┌───────────────────────────────────────────────────────────┼─┼────┐
│  Raspberry Pi (エッジ)                                    │ │    │
│                                                           │ │    │
│  lider_module_node ──── /scan ────────────────────────────┘ │    │
│  (USB CDC /dev/ttyACM0) /lider/pointcloud2                  │    │
│                         /lider/imu                          │    │
│                                ↑                            │    │
│  LiderModule                   │                            │    │
│  (XIAO ESP32-C3 +              │                            │    │
│   YDLIDAR T-mini Pro +         │                            │    │
│   STS3215 チルトサーボ)         │                            │    │
│                                                             │    │
│  ws_odometry_publisher ──── /odom ──────────────────────────┘    │
│  ws_motor_controller ◄──── /cmd_vel ──────────────────────────── │
│  ws_camera_bridge ──── /camera/image_raw/compressed ─────────── │
│                                                                   │
│  edge_services (ホスト側 Python)                                  │
│  ├ motor_service (ws :8001) ── PR2040 ── 4× DC モーター           │
│  ├ odometry_service (ws :8002) ── エンコーダー読み取り             │
│  └ camera_service (ws :8003) ── Raspberry Pi カメラ              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. ハードウェア要件

### 2.1 エッジ (Raspberry Pi)

| コンポーネント | 仕様 | 接続 |
|---|---|---|
| **Raspberry Pi 4 / 5** | 4GB RAM 以上推奨 | — |
| **Raspberry Pi Camera Module 3** | HQ カメラ / 固定フォーカス | CSI |
| **PR2040 モータードライバー** | I2C addr 0x60、4ch PWM | I2C-1 (`/dev/i2c-1`) |
| **DC モーター × 4** | エンコーダー付き、720 CPR | PR2040 ドライバー経由 |
| **LiderModule** | XIAO ESP32-C3 + T-mini Pro + STS3215 | USB (`/dev/ttyACM0`) |

#### LiderModule 詳細

| コンポーネント | 型番 | 仕様 |
|---|---|---|
| マイコン | Seeed XIAO ESP32-C3 | RISC-V 160 MHz, 400 KB RAM, USB CDC |
| LiDAR | YDLIDAR T-mini Pro | 360°, 4000 pts/s, 0.02–12 m, 6 Hz 回転 |
| チルトサーボ | FEETECH STS3215 | 7.4 V, 12 bit エンコーダー, 1 Mbps TTL |
| IMU | MPU6050 | 6 DoF, DMP, I2C addr 0x68 |
| チルト範囲 | −45° 〜 +45° | サーボ ID = 1, センター位置 = 2048 |

### 2.2 サーバー (Ubuntu デスクトップ)

| コンポーネント | 仕様 |
|---|---|
| **CPU** | x86_64、マルチコア (ROS2 + YOLO 同時実行) |
| **GPU** | NVIDIA RTX 4060 (YOLO 推論、CUDA 必須) |
| **RAM** | 16 GB 以上推奨 |
| **OS** | Ubuntu 24.04 (Noble) |

---

## 3. ソフトウェア要件

### 3.1 エッジ (Raspberry Pi)

#### OS・ランタイム

| ソフトウェア | バージョン | 役割 |
|---|---|---|
| Raspberry Pi OS (64 bit) | Bookworm | ベース OS |
| Python | 3.11+ | edge_services、スクリプト |
| Docker | 最新 | ROS2 コンテナ実行 |
| Docker Compose | v2 | コンテナ管理 |

#### Docker コンテナ (ROS2)

| ソフトウェア | バージョン | 役割 |
|---|---|---|
| ROS 2 Jazzy | ros:jazzy-ros-base | ROS2 ベース |
| ros-jazzy-nav2-bringup | — | Nav2 スタック |
| ros-jazzy-image-transport | — | カメラトランスポート |
| ros-jazzy-cv-bridge | — | 画像変換 |
| pyserial | ≥ 3.5 | LiderModule USB CDC 通信 |
| numpy | ≥ 1.21 | 座標変換 |
| smbus2 | ≥ 0.4 | I2C 通信 |
| websockets | ≥ 10.0 | edge_services WebSocket |
| openai | 最新 | LLM クライアント |

#### edge_services (ホスト側 Python)

| ソフトウェア | バージョン |
|---|---|
| smbus2 | ≥ 0.4.0 |
| websockets | ≥ 10.0 |
| opencv-python | ≥ 4.5.0 |
| numpy | ≥ 1.20.0 |

#### LiderModule ファームウェア

| ソフトウェア | バージョン |
|---|---|
| Arduino IDE | 2.x |
| Seeed XIAO ESP32-C3 ボードパッケージ | 最新 |
| SCServo ライブラリ | 最新 |
| MPU6050 (i2cdevlib DMP版) | 最新 |
| **ビルド設定** | USB CDC On Boot: **Enabled** |

### 3.2 サーバー (Ubuntu デスクトップ)

| ソフトウェア | バージョン | 役割 |
|---|---|---|
| ROS 2 Jazzy | ros:jazzy | ROS2 スタック |
| slam_toolbox | ros-jazzy-slam-toolbox | 2D LiDAR SLAM |
| Nav2 | ros-jazzy-nav2-bringup 他 | 自律走行スタック |
| Ollama | 最新 | ローカル LLM サーバー |
| qwen3.5:9b-nav | Ollama モデル | ナビゲーション推論 |
| YOLO (ultralytics) | yolov8s.pt | 物体検出 |
| rmw_zenoh_cpp | ros-jazzy | エッジ↔サーバー通信 |
| image_transport | ros-jazzy | 画像圧縮転送 |

---

## 4. ROS2 トピック・インターフェース

### 4.1 エッジ発行 (RPi → Server via Zenoh)

| トピック | 型 | 発行元 | 周期 | 説明 |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `lider_module_node` | ~5 Hz | tilt=0° 水平 360° スキャン |
| `/lider/pointcloud2` | `sensor_msgs/PointCloud2` | `lider_module_node` | 30 秒ごと | DOWN_ANGLES 24 ステップ 3D 点群 |
| `/lider/imu` | `sensor_msgs/Imu` | `lider_module_node` | 50 Hz | MPU6050 DMP 姿勢 (pitch/roll/yaw) |
| `/odom` | `nav_msgs/Odometry` | `ws_odometry_publisher` | 50 Hz | ホイールエンコーダー里程計 |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | `ws_camera_bridge` | 30 Hz | カメラ映像 (JPEG 圧縮) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `ws_camera_bridge` | 30 Hz | カメラ内部パラメーター |
| `/tf` | `tf2_msgs/TFMessage` | `robot_state_publisher` | 30 Hz | odom → base_footprint → base_link |

### 4.2 サーバー発行 (Server → RPi via Zenoh)

| トピック | 型 | 発行元 | 説明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / `llm_nav_controller` | 速度コマンド |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `llm_nav_controller` | Nav2 ゴール |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | 5 cm 解像度 OGM |
| `/tf` | `tf2_msgs/TFMessage` | `slam_toolbox` | map → odom |

### 4.3 TF フレームツリー

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── camera_link
            │   └── camera_optical_frame
            ├── laser_frame          ← LiderModule チルト軸中心
            │   └── imu_link         ← MPU6050
            └── [wheel links]
```

---

## 5. デバイスマッピング

### 5.1 Raspberry Pi デバイス

| デバイス | 用途 | 備考 |
|---|---|---|
| `/dev/ttyACM0` | LiderModule (XIAO ESP32-C3 USB CDC) | 921600 baud, DTR=False |
| `/dev/i2c-1` | PR2040 モータードライバー | addr 0x60 |
| `/dev/video0` | Raspberry Pi Camera Module 3 | v4l2 |
| `/dev/ttyUSB0` | 予備 (旧 YDLIDAR 等) | 現在未使用 |

### 5.2 Docker デバイスマウント

```yaml
devices:
  - /dev/i2c-1:/dev/i2c-1
  - /dev/video0:/dev/video0
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/ttyUSB1:/dev/ttyUSB1
  - /dev/ttyACM0:/dev/ttyACM0     # LiderModule
```

---

## 6. ロボット物理パラメーター

| パラメーター | 値 | 単位 |
|---|---|---|
| ホイールベース | 0.16 | m |
| ホイール半径 | 0.033 | m |
| エンコーダー分解能 | 720 | CPR |
| 最大直進速度 | 0.5 | m/s |
| 最大旋回速度 | 2.0 | rad/s |
| base_link 高さ | 0.033 | m (base_footprint から) |
| laser_frame 位置 | x=0.05, z=0.08 | m (base_link から) |

---

## 7. SLAM パラメーター (slam_toolbox)

| パラメーター | 値 | 説明 |
|---|---|---|
| `scan_topic` | `/scan` | LiderModule からの LaserScan |
| `resolution` | 0.05 m | マップ解像度 (5 cm) |
| `max_laser_range` | 12.0 m | T-mini Pro 最大レンジ |
| `mode` | mapping | オンライン非同期マッピング |
| `minimum_travel_distance` | 0.10 m | キーフレーム追加しきい値 |
| `minimum_travel_heading` | 0.087 rad | キーフレーム追加しきい値 (~5°) |
| `map_update_interval` | 3.0 s | Nav2 向けマップ配信間隔 |
| `solver_plugin` | CeresSolver | 最適化ソルバー |

---

## 8. LiderModule スキャンパラメーター

| パラメーター | 値 | 説明 |
|---|---|---|
| `port` | `/dev/ttyACM0` | USB CDC シリアルポート |
| `scan_frame_id` | `laser_frame` | LaserScan/PointCloud2 フレーム |
| `imu_frame_id` | `imu_link` | IMU フレーム |
| `min_range_m` | 0.02 | 最小有効距離 |
| `max_range_m` | 12.0 | 最大有効距離 |
| `laser_scan_bins` | 720 | LaserScan 角度分解能 (0.5°/bin) |
| `slice_timeout_s` | 4.0 | スライス受信タイムアウト |
| `pointcloud_interval_s` | 30.0 | 3D スキャン周期 (0 で無効) |

### DOWN_ANGLES パターン (24 ステップ)

```python
DOWN_ANGLES = [
    0.0,   -15.0, -30.0, -45.0,
    -42.5, -27.5, -12.5,   0.0,
    -5.0,  -20.0, -35.0, -45.0,
    -37.5, -22.5,  -7.5,   0.0,
    -10.0, -25.0, -40.0, -45.0,
    -32.5, -17.5,  -2.5,   0.0,
]
```

実測: 24 スライス → 約 1925 点、所要時間 約 8.5 秒

---

## 9. WebSocket サービス (edge_services)

| サービス | ポート | 役割 |
|---|---|---|
| `motor_service.py` | ws://localhost:8001 | `/cmd_vel` → PR2040 → モーター |
| `odometry_service.py` | ws://localhost:8002 | エンコーダー → `/odom` |
| `camera_service.py` | ws://localhost:8003 | カメラ映像 → `/camera/image_raw/compressed` |

---

## 10. 起動手順

### 10.1 Raspberry Pi (エッジ)

```bash
# 1. edge_services 起動
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py &

# 2. ROS2 コンテナ起動
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2 bash -c \
  "source /ros2_ws/install/setup.bash && \
   ros2 launch mobile_robot_edge ws_edge_bringup.launch.py"
```

### 10.2 Ubuntu サーバー

```bash
# Ollama 起動 (別ターミナル)
ollama serve &
ollama pull qwen3.5:9b-nav

# ROS2 サーバー起動
cd ~/ros2_ws
colcon build --packages-select mobile_robot_server
source install/setup.bash
ros2 launch mobile_robot_server server_bringup.launch.py

# 自律マッピング有効化 (オプション)
ros2 launch mobile_robot_server server_bringup.launch.py autonomous_mapping:=true
```

### 10.3 ビルド (変更後)

```bash
# Docker コンテナ内でビルド
cd /home/pi/MobileRobot/ros2_docker
docker compose run --rm ros2 bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   cd /ros2_ws && \
   colcon build --packages-select mobile_robot_edge mobile_robot_server"
```

---

## 11. ファイル構成

```
MobileRobot/
├── ros2_docker/
│   ├── Dockerfile                  # ROS2 Jazzy コンテナ定義
│   ├── docker-compose.yml          # デバイスマウント・ボリューム設定
│   └── ros2_workspace/             # colcon ビルド出力 (install/)
│
├── mobile_robot_edge/              # ROS2 エッジパッケージ
│   ├── mobile_robot_edge/
│   │   ├── lider_module_node.py    # LiderModule USB CDC → /scan, /lider/*
│   │   ├── ws_motor_controller.py  # /cmd_vel → WebSocket → motor_service
│   │   ├── ws_odometry_publisher.py# WebSocket → /odom
│   │   └── ws_camera_bridge.py     # WebSocket → /camera/*
│   └── launch/
│       └── ws_edge_bringup.launch.py
│
├── mobile_robot_server/            # ROS2 サーバーパッケージ
│   ├── mobile_robot_server/
│   │   ├── llm_nav_controller.py   # YOLO + Ollama LLM ナビゲーション
│   │   └── autonomous_mapping.py   # フロンティア探索 + LLM 判断
│   ├── config/
│   │   ├── slam_toolbox_params.yaml
│   │   ├── nav2_params.yaml
│   │   └── rtabmap_params.yaml
│   └── launch/
│       └── server_bringup.launch.py
│
├── edge_services/                  # ホスト側 WebSocket サービス
│   ├── motor_service.py            # ws :8001
│   ├── odometry_service.py         # ws :8002
│   ├── camera_service.py           # ws :8003
│   └── launch_all.py
│
└── LiderModule/                    # LiderModule ハードウェア一式
    ├── SPEC.md                     # ハードウェア・ファームウェア仕様書
    ├── firmware/lidar_tilt_3d/
    │   └── lidar_tilt_3d.ino       # XIAO ESP32-C3 ファームウェア
    ├── host/rpi_tilt_3d.py         # スタンドアロン 3D スキャンツール
    ├── docs/USB_CDC_COMMAND_MANUAL.md
    └── 計測パターン.csv
```

---

## 12. 既知の制約・注意事項

| 項目 | 内容 |
|---|---|
| `/scan` レート | ~5 Hz (LiDAR 6 Hz × 1 スキャン/コマンド)。slam_toolbox は `minimum_time_interval: 0.5s` で許容範囲内 |
| LiDAR モーター起動遅延 | CMD_SCAN_STOP 直後の CMD_SCAN_START は初回スキャン取得が遅れる場合あり |
| IMU yaw | MPU6050 DMP 6 軸のため絶対方位不可。相対角として扱う |
| サーボ安定待ち | `SCAN_SETTLE_DISABLED=1` (待ちなし)。障害物検出に有効だが点群精度は低下 |
| PointCloud2 の座標 | `/scan` (tilt=0°) との整合性は IMU 補正前提。動的シーンでは使用注意 |
| Docker デバイス | `docker-compose.yml` に `/dev/ttyACM0` 要記載。LiderModule 未接続時はノードがエラー終了 |
| USB テキストデバッグ | ファームウェアの `USB_TEXT_DEBUG_MODE=1` 時はバイナリプロトコル無効。運用時は `0` に設定 |
