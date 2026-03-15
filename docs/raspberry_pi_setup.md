# Raspberry Pi セットアップガイド

Mobile Robot プロジェクトの Raspberry Pi 側のセットアップ手順です。

## 目次

1. [システム要件](#システム要件)
2. [基本システムセットアップ](#基本システムセットアップ)
3. [Docker インストール](#docker-インストール)
4. [Python 環境セットアップ](#python-環境セットアップ)
5. [プロジェクトのクローン](#プロジェクトのクローン)
6. [ハードウェア接続の確認](#ハードウェア接続の確認)
7. [Edge Services のセットアップ](#edge-services-のセットアップ)
8. [ROS 2 Docker 環境のセットアップ](#ros-2-docker-環境のセットアップ)
9. [システムの起動](#システムの起動)
10. [動作確認](#動作確認)
11. [トラブルシューティング](#トラブルシューティング)

---

## システム要件

- **ハードウェア**: Raspberry Pi 4 以上推奨
- **OS**: Debian 13 (Trixie) または Raspberry Pi OS
- **RAM**: 4GB 以上推奨
- **ストレージ**: 16GB 以上の空き容量
- **接続**: PR2040 モータードライバ（USB経由）

---

## 基本システムセットアップ

### 1. システムの更新

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. 必要な基本パッケージのインストール

```bash
sudo apt install -y \
    git \
    python3 \
    python3-pip \
    python3-venv \
    build-essential \
    curl \
    nano \
    v4l-utils
```

---

## Docker インストール

ROS 2 を Docker コンテナで実行するため、Docker をインストールします。

### 1. Docker のインストール

```bash
# Docker の公式インストールスクリプトをダウンロード
curl -fsSL https://get.docker.com -o get-docker.sh

# インストールスクリプトを実行
sudo sh get-docker.sh

# 現在のユーザーを docker グループに追加
sudo usermod -aG docker $USER

# 変更を反映（ログアウト/ログインまたは再起動が必要）
newgrp docker
```

### 2. Docker のバージョン確認

```bash
docker --version
# 出力例: Docker version 29.3.0, build ...
```

### 3. Docker Compose のインストール確認

```bash
docker compose version
# 出力例: Docker Compose version v2.x.x
```

---

## Python 環境セットアップ

Edge Services で使用する Python パッケージをインストールします。

### 1. pip のアップグレード

```bash
python3 -m pip install --upgrade pip --break-system-packages
```

### 2. 必要な Python パッケージのインストール

```bash
pip3 install --break-system-packages \
    pyserial \
    websockets \
    smbus2
```

**注意**: Debian 13 では `--break-system-packages` フラグが必要です。

---

## プロジェクトのクローン

### 1. GitHub からプロジェクトをクローン

```bash
cd ~
git clone https://github.com/rx178nwj/MobileRobot.git
cd MobileRobot
```

### 2. プロジェクト構造の確認

```bash
tree -L 2 -d
```

主要なディレクトリ:
```
MobileRobot/
├── edge_services/          # Python WebSocket サービス
│   ├── hardware/          # ハードウェアドライバ
│   ├── config/            # 設定ファイル
│   └── logs/              # ログファイル（自動生成）
├── ros2_docker/           # ROS 2 Docker 環境
├── mobile_robot_edge/     # ROS 2 パッケージ（WebSocket ブリッジ）
└── MotorDriver/           # PR2040 ファームウェア（参照用）
```

---

## ハードウェア接続の確認

### 1. PR2040 USB 接続の確認

PR2040 モータードライバを USB で接続し、デバイスが認識されているか確認します。

```bash
ls -l /dev/ttyACM*
```

出力例:
```
crw-rw---- 1 root dialout 166, 0 Mar 15 10:00 /dev/ttyACM0
```

### 2. USB シリアルポートへのアクセス権限

```bash
# dialout グループに追加（既に追加されている場合もあります）
sudo usermod -aG dialout $USER

# 変更を反映
newgrp dialout
```

### 3. シリアルポートのテスト

```bash
cd ~/MobileRobot
python3 test_pr2040_usb_fixed.py --test basic
```

期待される出力:
```
✅ Connected to PR2040 at /dev/ttyACM0
✅ Temperature: XX.XX°C
✅ Encoders: (0, 0, 0, 0)
```

---

## Edge Services のセットアップ

Edge Services は、ハードウェアと ROS 2 の間で WebSocket 通信を提供します。

### 1. 設定ファイルの確認

```bash
cat edge_services/config/robot_config.json
```

主要な設定項目:
- `encoder_cpr`: 827.2 (キャリブレーション済み)
- `usb_port`: "/dev/ttyACM0"
- `motor_service.port`: 8001
- `odometry_service.port`: 8002

### 2. Edge Services のテスト

```bash
cd ~/MobileRobot
python3 test_usb_services.py
```

全てのテストが PASSED になることを確認:
```
✅ Basic USB Driver........................ PASSED
✅ Motor Service........................... PASSED
✅ Odometry Service........................ PASSED
✅ Motor Movement.......................... PASSED
```

---

## ROS 2 Docker 環境のセットアップ

### 1. Docker イメージのビルド

```bash
cd ~/MobileRobot/ros2_docker
docker compose build
```

ビルドには数分かかります。完了すると:
```
Successfully tagged mobile-robot-ros2:jazzy
```

### 2. ROS 2 パッケージのビルド

```bash
# Docker コンテナ内で ROS 2 パッケージをビルド
docker compose run --rm ros2 bash -c "
    cd /home/pi/ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install
"
```

ビルドが成功すると:
```
Summary: X packages finished [Xs]
```

---

## システムの起動

### 起動手順

完全なシステムを起動するには、3つのターミナルを使用します。

#### ターミナル 1: Edge Services の起動

```bash
cd ~/MobileRobot/edge_services
python3 launch_all.py
```

出力例:
```
============================================================
Starting Edge Services
============================================================
✓ Motor Control Service enabled
✓ Odometry Service enabled
✓ Camera Service enabled
============================================================
All services started successfully
Press Ctrl+C to stop
```

#### ターミナル 2: ROS 2 システムの起動

```bash
cd ~/MobileRobot/ros2_docker
docker compose run --rm ros2 bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /home/pi/ros2_ws/install/setup.bash && \
    ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
"
```

出力例:
```
[INFO] [ws_motor_controller]: Connected to motor service ws://localhost:8001
[INFO] [ws_odometry_publisher]: Connected to odometry service ws://localhost:8002
[INFO] [robot_state_publisher]: Publishing transforms
```

#### ターミナル 3: テストコマンドの実行（オプション）

```bash
# 別のターミナルで ROS 2 コンテナに接続
cd ~/MobileRobot/ros2_docker
docker compose run --rm ros2 bash

# コンテナ内で環境をセットアップ
source /opt/ros/jazzy/setup.bash
source /home/pi/ros2_ws/install/setup.bash

# トピックの確認
ros2 topic list

# cmd_vel を送信（前進）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# オドメトリの確認
ros2 topic echo /odom --once
```

---

## 動作確認

### 1. WebSocket 接続の確認

Edge Services が起動している状態で:

```bash
# モーターサービスへの接続テスト
cd ~/MobileRobot
python3 test_move_forward.py
```

### 2. エンコーダの読み取り確認

```bash
python3 -c "
import sys
sys.path.insert(0, '/home/pi/MobileRobot/edge_services')
from hardware.pr2040_usb_driver import PR2040USBDriver

driver = PR2040USBDriver()
encoders = driver.read_all_encoders()
print(f'Encoders: {encoders}')
driver.close()
"
```

### 3. モーターの動作確認

```bash
cd ~/MobileRobot
python3 test_motors_complete.py --test direct
```

**注意**: 現在、モーター 0 と 1 のみ動作確認済み（モーター 2, 3 はハードウェア問題）

### 4. ROS 2 トピックの確認

ROS 2 が起動している状態で、別のターミナルから:

```bash
cd ~/MobileRobot/ros2_docker
docker compose run --rm ros2 bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /home/pi/ros2_ws/install/setup.bash && \
    ros2 topic list
"
```

期待される出力:
```
/cmd_vel
/odom
/tf
/tf_static
```

---

## トラブルシューティング

### 問題 1: `/dev/ttyACM0` が見つからない

**症状**:
```
FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyACM0'
```

**解決方法**:
1. PR2040 が USB で接続されているか確認
2. デバイスを確認: `ls -l /dev/ttyACM*`
3. USB ケーブルを抜き差しして再接続
4. `dmesg | tail` でカーネルログを確認

---

### 問題 2: Permission denied エラー

**症状**:
```
PermissionError: [Errno 13] Permission denied: '/dev/ttyACM0'
```

**解決方法**:
```bash
# dialout グループに追加
sudo usermod -aG dialout $USER

# ログアウト/ログインまたは
newgrp dialout

# または再起動
sudo reboot
```

---

### 問題 3: WebSocket 接続エラー

**症状**:
```
ERROR: Cannot connect to ws://localhost:8001
```

**解決方法**:
1. Edge Services が起動しているか確認
2. ポートが使用中か確認: `netstat -tuln | grep 800`
3. Edge Services を再起動

```bash
# 既存のプロセスを停止
pkill -f launch_all.py

# 再起動
cd ~/MobileRobot/edge_services
python3 launch_all.py
```

---

### 問題 4: Docker コンテナが起動しない

**症状**:
```
ERROR: Cannot connect to the Docker daemon
```

**解決方法**:
```bash
# Docker サービスの状態確認
sudo systemctl status docker

# Docker サービスの起動
sudo systemctl start docker

# 自動起動の有効化
sudo systemctl enable docker
```

---

### 問題 5: モーターが動かない

**症状**: エンコーダ値が変化しない

**確認事項**:
1. PR2040 への電源供給を確認
2. モータードライバの LED を確認
3. USB 接続を確認
4. テストスクリプトで確認:

```bash
python3 test_motors_complete.py --test direct
```

**既知の問題**:
- モーター 2 と 3 はハードウェアの問題で動作しません
- モーター 0 と 1 は正常に動作します

---

### 問題 6: Python パッケージのインポートエラー

**症状**:
```
ModuleNotFoundError: No module named 'websockets'
```

**解決方法**:
```bash
cd ~/MobileRobot/edge_services
pip3 install --break-system-packages -r requirements.txt
```

---

## システムの自動起動設定（オプション）

### Systemd サービスの作成

Edge Services を systemd サービスとして登録し、自動起動できるようにします。

#### 1. サービスファイルの作成

```bash
sudo nano /etc/systemd/system/edge-services.service
```

内容:
```ini
[Unit]
Description=Mobile Robot Edge Services
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/MobileRobot/edge_services
ExecStart=/usr/bin/python3 /home/pi/MobileRobot/edge_services/launch_all.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

#### 2. サービスの有効化と起動

```bash
# サービスの再読み込み
sudo systemctl daemon-reload

# サービスの有効化（起動時に自動起動）
sudo systemctl enable edge-services

# サービスの起動
sudo systemctl start edge-services

# ステータスの確認
sudo systemctl status edge-services
```

#### 3. ログの確認

```bash
# リアルタイムログの確認
sudo journalctl -u edge-services -f

# 最新のログを表示
sudo journalctl -u edge-services -n 50
```

---

## 参考資料

### プロジェクト関連ドキュメント

- [CommandReference.md](../MotorDriver/Firmware/PR2040_MotorDriver/CommandReference.md) - PR2040 通信プロトコル仕様
- [README.md](../README.md) - プロジェクト概要

### テストスクリプト

- `test_usb_services.py` - 総合テストスイート
- `test_motors_complete.py` - モーター個別テスト
- `test_pr2040_usb_fixed.py` - USB 通信テスト
- `test_move_forward.py` - WebSocket 経由移動テスト

### 設定ファイル

- `edge_services/config/robot_config.json` - ロボット設定
- `ros2_docker/docker-compose.yml` - Docker 設定
- `ros2_docker/Dockerfile` - Docker イメージ定義

---

## 次のステップ

セットアップが完了したら:

1. **キャリブレーション**: ロボットの実際の動作に基づいてパラメータを調整
2. **ナビゲーション**: ROS 2 Nav2 スタックの統合
3. **センサー統合**: IMU、カメラなどの追加センサーの統合
4. **自律動作**: 自律ナビゲーションと障害物回避の実装

---

## サポート

問題が発生した場合は、GitHub Issues で報告してください:
https://github.com/rx178nwj/MobileRobot/issues

---

**最終更新日**: 2026-03-15
