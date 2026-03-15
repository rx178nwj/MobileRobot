# Edge Services

シンプルなPythonベースのモバイルロボットエッジ制御サービス

## 概要

Raspberry Pi上で動作する軽量なロボット制御サービス群です。ROS 2を使用せず、WebSocket通信で外部PCと連携します。

### アーキテクチャ

```
外部PC (ROS 2)  ←→  WebSocket  ←→  Raspberry Pi (Edge Services)
                                          ↓ I2C
                                    PR2040 Motor Driver
```

## 機能

### 1. モーター制御サービス (`motor_service.py`)
- WebSocketで速度指令を受信
- 差動駆動キネマティクス
- I2C経由でPR2040モータードライバ制御
- 安全タイムアウト機能

**WebSocket**: `ws://raspberrypi.local:8001`

**メッセージフォーマット**:
```json
{
  "type": "cmd_vel",
  "linear": 0.2,   // m/s
  "angular": 0.5   // rad/s
}
```

### 2. オドメトリサービス (`odometry_service.py`)
- エンコーダ読み取り (I2C)
- オドメトリ計算 (位置・速度)
- WebSocketで配信 (50Hz)

**WebSocket**: `ws://raspberrypi.local:8002`

**メッセージフォーマット**:
```json
{
  "type": "odom",
  "timestamp": 1234567.89,
  "pose": {
    "x": 0.0,      // meters
    "y": 0.0,      // meters
    "theta": 0.0   // radians
  },
  "twist": {
    "linear": 0.0,  // m/s
    "angular": 0.0  // rad/s
  },
  "encoders": [0, 0, 0, 0]
}
```

### 3. カメラサービス (`camera_service.py`)
- Raspberry Pi Camera Module 3から画像取得
- オートフォーカス無効化 (SLAM用)
- JPEG圧縮配信
- WebSocket配信 (30fps)

**WebSocket**: `ws://raspberrypi.local:8003`

**メッセージフォーマット**: バイナリ (ヘッダー8バイト + JPEGデータ)

## ファイル構成

```
edge_services/
├── hardware/
│   ├── __init__.py
│   └── pr2040_driver.py      # PR2040 I2C通信ライブラリ
├── config/
│   └── robot_config.json     # ロボット設定
├── logs/                     # ログファイル
├── motor_service.py          # モーター制御サービス
├── odometry_service.py       # オドメトリサービス
├── camera_service.py         # カメラサービス
├── launch_all.py             # 全サービス起動スクリプト
├── requirements.txt          # Python依存関係
└── README.md                 # 本ドキュメント
```

## セットアップ

### 1. 依存パッケージのインストール

```bash
cd /home/pi/MobileRobot/edge_services

# Pythonパッケージ
pip3 install -r requirements.txt

# システムパッケージ
sudo apt install -y python3-opencv i2c-tools v4l-utils
```

### 2. I2C有効化

```bash
# raspi-configでI2Cを有効化
sudo raspi-config
# Interface Options → I2C → Yes

# I2Cデバイス確認
i2cdetect -y 1
# 0x60にPR2040が表示されるはず
```

### 3. カメラ設定

```bash
# カメラデバイス確認
ls -l /dev/video0

# カメラ情報確認
v4l2-ctl -d /dev/video0 --all

# フォーカス値の調整（オプション）
v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=450
```

### 4. 設定ファイル編集

`config/robot_config.json`を編集してロボットパラメータを調整:

```json
{
  "robot": {
    "wheel_base": 0.16,     // 車輪間距離 (m)
    "wheel_radius": 0.033,  // 車輪半径 (m)
    "encoder_cpr": 720      // エンコーダ分解能
  }
}
```

## 起動方法

### すべてのサービスを起動

```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

### 個別サービスの起動

```bash
# モーター制御のみ
python3 motor_service.py

# オドメトリのみ
python3 odometry_service.py

# カメラのみ
python3 camera_service.py
```

### systemdサービスとして登録（自動起動）

```bash
# サービスファイル作成
sudo nano /etc/systemd/system/edge-services.service
```

```ini
[Unit]
Description=Mobile Robot Edge Services
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/MobileRobot/edge_services
ExecStart=/usr/bin/python3 /home/pi/MobileRobot/edge_services/launch_all.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# サービス有効化
sudo systemctl daemon-reload
sudo systemctl enable edge-services
sudo systemctl start edge-services

# ステータス確認
sudo systemctl status edge-services

# ログ確認
sudo journalctl -u edge-services -f
```

## テスト

### WebSocketテスト (Python)

```python
import asyncio
import websockets
import json

async def test_motor():
    uri = "ws://localhost:8001"
    async with websockets.connect(uri) as ws:
        # 前進指令
        cmd = {"type": "cmd_vel", "linear": 0.2, "angular": 0.0}
        await ws.send(json.dumps(cmd))
        response = await ws.recv()
        print(f"Motor response: {response}")

async def test_odometry():
    uri = "ws://localhost:8002"
    async with websockets.connect(uri) as ws:
        # オドメトリ受信
        for i in range(10):
            msg = await ws.recv()
            data = json.loads(msg)
            print(f"Odom: x={data['pose']['x']:.3f} y={data['pose']['y']:.3f}")

asyncio.run(test_motor())
asyncio.run(test_odometry())
```

### I2C通信テスト

```bash
# PR2040ドライバテスト
cd /home/pi/MobileRobot/edge_services
python3 -c "from hardware.pr2040_driver import PR2040Driver; \
driver = PR2040Driver(); \
print('Temp:', driver.read_temperature()); \
print('Encoders:', driver.read_all_encoders()); \
driver.close()"
```

## トラブルシューティング

### I2C通信エラー

```bash
# I2Cデバイス確認
i2cdetect -y 1

# PR2040が0x60に表示されない場合:
# - 配線を確認
# - PR2040の電源を確認
# - I2Cプルアップ抵抗を確認
```

### カメラが起動しない

```bash
# カメラデバイス確認
ls -l /dev/video*

# カメラが検出されない場合:
sudo vcgencmd get_camera

# カメラを有効化
sudo raspi-config
# Interface Options → Camera → Yes
# 再起動が必要
```

### WebSocketに接続できない

```bash
# ポートが開いているか確認
sudo netstat -tuln | grep 800

# ファイアウォール確認（Debian/Raspberry Pi OSではデフォルトで無効）
sudo ufw status

# サービスログ確認
tail -f logs/edge_services.log
```

## 外部PCとの連携 (ROS 2ブリッジ)

外部PC側で ROS 2 ブリッジノードを作成すると、ROS 2 Navigation Stackと連携できます。

### ROS 2ブリッジノード例

```python
# 外部PC (ROS 2) 側
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import asyncio
import websockets

class EdgeBridgeNode(Node):
    def __init__(self):
        super().__init__('edge_bridge')

        # ROS 2トピック
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # WebSocket接続 (Raspberry Pi)
        self.motor_ws = "ws://raspberrypi.local:8001"
        self.odom_ws = "ws://raspberrypi.local:8002"

    # ... 実装 ...
```

## パラメータ調整

### フォーカス値の最適化

ロボットの動作環境に応じてカメラのフォーカス値を調整:

- **近距離** (20-50cm): `focus_value: 200-350`
- **中距離** (50cm-2m): `focus_value: 400-500`
- **遠距離** (1m-∞): `focus_value: 500-650`

### PIDゲイン調整

PR2040ドライバでPIDゲインを調整可能:

```python
from hardware.pr2040_driver import PR2040Driver

driver = PR2040Driver()

# 速度PIDゲイン設定 (wheel_index, kp, ki, kd)
# デフォルト: kp=1.0, ki=0.05, kd=0.0
kp, ki, kd = 1.5, 0.1, 0.0
data = struct.pack('<Bfff', 0, kp, ki, kd)
driver._write_register(driver.REG_SET_VEL_PID, list(data))
```

## ライセンス

MIT

## 作者

rx178nwj
