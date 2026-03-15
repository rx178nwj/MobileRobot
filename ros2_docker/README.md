# ROS 2 Jazzy Docker Setup

このディレクトリは、Raspberry Pi上でROS 2 JazzyをDockerコンテナとして実行するための設定ファイルを含んでいます。

## 概要

Debian 13 (trixie) では ROS 2 のネイティブインストールに依存関係の問題があるため、Docker環境でROS 2を実行します。

## ファイル構成

- `Dockerfile` - ROS 2 Jazzy ベースイメージの定義
- `docker-compose.yml` - コンテナ設定とデバイスマッピング
- `README.md` - 本ドキュメント

## 使用方法

### イメージのビルド

```bash
cd /home/pi/MobileRobot/ros2_docker
docker-compose build
```

初回ビルドは15-30分程度かかる場合があります。

### コンテナの起動

```bash
# コンテナを起動してbashシェルに入る
docker-compose run --rm ros2

# または、バックグラウンドで起動
docker-compose up -d
docker exec -it mobile_robot_ros2 bash
```

### ROS 2の動作確認

コンテナ内で:

```bash
# ROS 2のバージョン確認
ros2 --version

# ノードのリスト表示
ros2 node list

# トピックのリスト表示
ros2 topic list
```

### ワークスペースのビルド

```bash
# コンテナ内で
cd /ros2_ws
colcon build --packages-select mobile_robot_edge
source install/setup.bash
```

### ハードウェアアクセス

以下のデバイスがコンテナにマウントされています:

- `/dev/i2c-1` - I2C通信 (PR2040との通信)
- `/dev/video0` - カメラデバイス (Raspberry Pi Camera Module 3)

#### I2C動作確認

```bash
# コンテナ内で
i2cdetect -y 1
# 0x60にPR2040が表示されるはず
```

#### カメラ動作確認

```bash
# コンテナ内で
v4l2-ctl -d /dev/video0 --list-formats-ext
```

## アーキテクチャ

### システム構成

```
┌─────────────────────────────────────────┐
│ Raspberry Pi (Debian 13)               │
│                                         │
│  ┌───────────────────────────────────┐ │
│  │ Docker Container (Ubuntu 24.04)   │ │
│  │                                   │ │
│  │  ┌─────────────────────────────┐ │ │
│  │  │ ROS 2 Jazzy                 │ │ │
│  │  │ - Navigation Stack          │ │ │
│  │  │ - SLAM Toolbox              │ │ │
│  │  │ - Robot State Publisher     │ │ │
│  │  └─────────────────────────────┘ │ │
│  │                                   │ │
│  │  ┌─────────────────────────────┐ │ │
│  │  │ mobile_robot_edge package   │ │ │
│  │  │ - Camera launch             │ │ │
│  │  │ - Edge bringup              │ │ │
│  │  └─────────────────────────────┘ │ │
│  └───────────────────────────────────┘ │
│           ↓ (device mapping)           │
│  ┌───────────────────────────────────┐ │
│  │ Edge Services (Python)            │ │
│  │ - motor_service.py   (WS:8001)    │ │
│  │ - odometry_service.py (WS:8002)   │ │
│  │ - camera_service.py  (WS:8003)    │ │
│  └───────────────────────────────────┘ │
│           ↓ I2C (/dev/i2c-1)           │
│  ┌───────────────────────────────────┐ │
│  │ PR2040 Motor Driver (0x60)        │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

### 通信フロー

1. **外部PC → ROS 2 (Docker内)**
   - ROS 2 ノード間通信 (DDS)
   - Navigation指令、SLAMデータなど

2. **ROS 2 (Docker内) → Edge Services**
   - WebSocket通信 (localhost:8001-8003)
   - cmd_vel指令、オドメトリ、カメラ画像

3. **Edge Services → PR2040**
   - I2C通信 (/dev/i2c-1, address 0x60)
   - モーター制御、エンコーダ読み取り

## トラブルシューティング

### イメージビルドエラー

```bash
# キャッシュをクリアして再ビルド
docker-compose build --no-cache
```

### デバイスアクセスエラー

```bash
# I2Cデバイスの権限確認
ls -l /dev/i2c-1

# 必要に応じて権限変更
sudo chmod 666 /dev/i2c-1
sudo chmod 666 /dev/video0
```

### コンテナが起動しない

```bash
# ログを確認
docker-compose logs

# 既存のコンテナを削除
docker-compose down
docker-compose up
```

## メモ

- コンテナは `privileged` モードで実行されます (ハードウェアアクセスのため)
- `network_mode: host` により、ホストのネットワークを共有します
- ROS 2のDDS通信は `ROS_DOMAIN_ID=0` で設定されています
