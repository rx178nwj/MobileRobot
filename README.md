# Mobile Robot

4輪駆動移動ロボットプロジェクト - ROS 2、Python WebSocket サービス、PR2040 モータードライバを使用

## プロジェクト概要

このプロジェクトは、Raspberry Pi と PR2040 マイコンを使用した移動ロボットシステムです。ROS 2 (Jazzy) を Docker コンテナで実行し、Python ベースの Edge Services を介してハードウェアと通信します。

### 主な特徴

- **ROS 2 Jazzy** - Docker コンテナで実行（Debian 13 ホスト）
- **WebSocket 通信** - ROS 2 と Edge Services 間の通信
- **USB シリアル制御** - PR2040 モータードライバとの通信
- **リアルタイムオドメトリ** - 50Hz でのエンコーダベース位置推定
- **差動駆動制御** - 4輪駆動の差動駆動キネマティクス
- **速度制御モード** - PR2040 ファームウェアによる PID 速度制御

## システムアーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 (Docker Container)                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ ws_motor_    │  │ ws_odometry_ │  │ robot_state_ │      │
│  │ controller   │  │ publisher    │  │ publisher    │      │
│  └──────┬───────┘  └───────▲──────┘  └──────────────┘      │
└─────────┼──────────────────┼────────────────────────────────┘
          │                  │
       WebSocket          WebSocket
       (port 8001)        (port 8002)
          │                  │
          ▼                  ▼
┌─────────────────────────────────────────────────────────────┐
│                      Edge Services (Host)                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Motor        │  │ Odometry     │  │ Camera       │      │
│  │ Service      │  │ Service      │  │ Service      │      │
│  └──────┬───────┘  └───────▲──────┘  └──────────────┘      │
└─────────┼──────────────────┼────────────────────────────────┘
          │                  │
       USB Serial        USB Serial
       (/dev/ttyACM0)    (/dev/ttyACM0)
          │                  │
          ▼                  ▼
┌─────────────────────────────────────────────────────────────┐
│                  PR2040 Motor Driver                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Motor 0  │  │ Motor 1  │  │ Motor 2  │  │ Motor 3  │   │
│  │ (動作OK) │  │ (動作OK) │  │ (故障)   │  │ (故障)   │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## ハードウェア

- **Raspberry Pi 4** - メインコントローラ
- **PR2040 (RP2040)** - モータードライバマイコン
- **JGA25-370 モーター** - ギア比 51:1、エンコーダ付き
- **車輪**: 直径 66mm (半径 33mm)
- **ホイールベース**: 160mm

### キャリブレーション値

- **エンコーダ CPR**: 827.2 counts/revolution（キャリブレーション済み）
- **ギア比**: 54.78:1（実測値、ファームウェアに組み込み済み）
- **車輪円周**: 212.06mm
- **変換係数**: 0.2564 mm/count

## ディレクトリ構成

```
MobileRobot/
├── edge_services/              # Python WebSocket サービス
│   ├── hardware/              # ハードウェアドライバ
│   │   ├── pr2040_usb_driver.py    # USB シリアルドライバ (使用中)
│   │   └── pr2040_driver.py        # I2C ドライバ (非推奨)
│   ├── config/                # 設定ファイル
│   │   └── robot_config.json  # ロボット設定
│   ├── motor_service.py       # モーター制御サービス (WS:8001)
│   ├── odometry_service.py    # オドメトリサービス (WS:8002)
│   ├── camera_service.py      # カメラサービス (WS:8003)
│   └── launch_all.py          # 全サービス起動スクリプト
│
├── ros2_docker/               # ROS 2 Docker 環境
│   ├── Dockerfile             # ROS 2 Jazzy イメージ
│   └── docker-compose.yml     # Docker Compose 設定
│
├── mobile_robot_edge/         # ROS 2 パッケージ
│   ├── mobile_robot_edge/
│   │   ├── ws_motor_controller.py      # /cmd_vel → WS ブリッジ
│   │   └── ws_odometry_publisher.py    # WS → /odom ブリッジ
│   ├── launch/
│   │   └── ws_edge_bringup.launch.py   # 起動ファイル
│   ├── urdf/
│   │   └── mobile_robot.urdf           # ロボットモデル
│   ├── package.xml
│   └── setup.py
│
├── MotorDriver/               # PR2040 ファームウェア
│   └── Firmware/
│       └── PR2040_MotorDriver/
│           ├── PR2040_MotorDriver.ino  # メインファームウェア
│           └── CommandReference.md     # 通信プロトコル仕様
│
├── docs/                      # ドキュメント
│   └── raspberry_pi_setup.md  # Raspberry Pi セットアップガイド
│
└── test_*.py                  # 各種テストスクリプト
```

## セットアップ

### クイックスタート

```bash
# 1. リポジトリのクローン
git clone https://github.com/rx178nwj/MobileRobot.git
cd MobileRobot

# 2. 詳細なセットアップ手順に従う
# docs/raspberry_pi_setup.md を参照してください
```

### 詳細なセットアップ

完全なセットアップ手順については、以下のドキュメントを参照してください:

📚 **[Raspberry Pi セットアップガイド](docs/raspberry_pi_setup.md)**

このガイドには以下が含まれます:
- システム要件
- Docker インストール
- Python 環境セットアップ
- ハードウェア接続確認
- Edge Services セットアップ
- ROS 2 Docker 環境セットアップ
- システムの起動方法
- トラブルシューティング

## 使い方

### システムの起動

#### 1. Edge Services の起動（ターミナル 1）

```bash
cd ~/MobileRobot/edge_services
python3 launch_all.py
```

#### 2. ROS 2 システムの起動（ターミナル 2）

```bash
cd ~/MobileRobot/ros2_docker
docker compose run --rm ros2 bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /home/pi/ros2_ws/install/setup.bash && \
    ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
"
```

#### 3. ロボットの制御（ターミナル 3）

```bash
cd ~/MobileRobot/ros2_docker
docker compose run --rm ros2 bash

# コンテナ内で
source /opt/ros/jazzy/setup.bash
source /home/pi/ros2_ws/install/setup.bash

# 前進コマンド
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# オドメトリの確認
ros2 topic echo /odom
```

### テスト

#### 総合テストスイート

```bash
cd ~/MobileRobot
python3 test_usb_services.py
```

#### モーター個別テスト

```bash
# 各モーターを個別にテスト（DIRECT モード）
python3 test_motors_complete.py --test direct

# 10cm 前進テスト（VELOCITY モード）
python3 test_motors_complete.py --test velocity
```

#### USB 通信テスト

```bash
# 基本的な USB 通信テスト
python3 test_pr2040_usb_fixed.py --test basic

# 10cm 移動テスト
python3 test_pr2040_usb_fixed.py --test move
```

## 通信プロトコル

### USB シリアルプロトコル

PR2040 との通信は以下のパケット形式を使用:

```
[0xAA][CMD][LEN][DATA...][CHECKSUM]
```

- **Header**: 0xAA（固定）
- **CMD**: コマンドバイト
- **LEN**: データ長
- **DATA**: コマンドデータ
- **CHECKSUM**: XOR チェックサム（CMD ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]）

詳細は [CommandReference.md](MotorDriver/Firmware/PR2040_MotorDriver/CommandReference.md) を参照。

### WebSocket プロトコル

#### モーター制御（ポート 8001）

```json
{
  "type": "cmd_vel",
  "linear": 0.1,    // m/s
  "angular": 0.5    // rad/s
}
```

#### オドメトリ（ポート 8002）

```json
{
  "type": "odom",
  "timestamp": 1234567.89,
  "pose": {
    "x": 0.0,       // meters
    "y": 0.0,       // meters
    "theta": 0.0    // radians
  },
  "twist": {
    "linear": 0.0,  // m/s
    "angular": 0.0  // rad/s
  },
  "encoders": [100, 100, 0, 0]
}
```

## 既知の問題

### ハードウェア

- **モーター 2 と 3**: ハードウェアの問題により動作しません（エンコーダ値が常に 0）
- **モーター 0 と 1**: 正常に動作します

### 対応予定

これらのハードウェア問題は、モータードライバボードまたは配線の問題と考えられます。

## 開発状況

### 完了

- ✅ PR2040 ファームウェア（USB シリアル + I2C）
- ✅ USB シリアルドライバ
- ✅ Edge Services（モーター制御、オドメトリ、カメラ）
- ✅ ROS 2 Docker 環境
- ✅ WebSocket ブリッジ
- ✅ 基本的な動作確認

### 今後の予定

- ⏳ IMU センサー統合
- ⏳ カメラ画像処理
- ⏳ ナビゲーションスタック（Nav2）
- ⏳ SLAM（地図作成と自己位置推定）
- ⏳ 自律ナビゲーション

## ライセンス

このプロジェクトは MIT ライセンスの下で公開されています。

## 貢献

プルリクエストを歓迎します。大きな変更の場合は、まず issue を開いて変更内容を議論してください。

## 作者

- GitHub: [@rx178nwj](https://github.com/rx178nwj)

## 謝辞

- ROS 2 コミュニティ
- Raspberry Pi Foundation
- Arduino/RP2040 コミュニティ

---

**最終更新日**: 2026-03-15
