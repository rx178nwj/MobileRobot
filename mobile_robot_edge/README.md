# mobile_robot_edge

ROS 2 Humbleパッケージ - モバイルロボットのエッジ側制御（Raspberry Pi）

## 概要

このパッケージはRaspberry Pi上で動作し、センサーデータの取得とモーター制御を担当します。
重い処理（SLAM、ナビゲーション等）は別のPCで実行されることを想定しています。

## ハードウェア要件

- **SBC**: Raspberry Pi 4/5 (Ubuntu 22.04)
- **カメラ**: Raspberry Pi Camera Module 3
- **モータードライバ**: PR2040 Motor Driver (I2C: 0x60)
- **モーター**: 4輪駆動DC モーター with エンコーダ

## 機能

### 1. カメラノード
- Raspberry Pi Camera Module 3からの画像配信
- **オートフォーカス無効化** (SLAM安定化のため)
- 圧縮画像配信 (ネットワーク帯域節約)
- トピック: `/camera/image_raw/compressed`

### 2. オドメトリ配信
- 車輪エンコーダからの位置推定
- トピック: `/odom` (nav_msgs/Odometry)
- TF配信: `odom` → `base_footprint`

### 3. モーター制御
- `/cmd_vel` 購読してモーター制御
- 差動駆動キネマティクス
- 安全タイムアウト機能

## ビルド方法

```bash
cd ~/ros2_ws
colcon build --packages-select mobile_robot_edge
source install/setup.bash
```

## 起動方法

### 全機能を一括起動
```bash
ros2 launch mobile_robot_edge edge_bringup.launch.py
```

### 個別起動

**カメラのみ:**
```bash
ros2 launch mobile_robot_edge camera.launch.py
```

**オドメトリのみ:**
```bash
ros2 run mobile_robot_edge odometry_publisher
```

**モーター制御のみ:**
```bash
ros2 run mobile_robot_edge motor_controller
```

## ハードウェア統合手順

現在、エンコーダ読み取りとモーター制御は**テンプレート実装**です。
実際のハードウェアを接続する際は、以下のファイルの`TODO`セクションを実装してください。

### 1. オドメトリ配信ノード

**ファイル**: `mobile_robot_edge/odometry_publisher.py`

**実装箇所:**

#### `init_hardware()` メソッド
```python
# TODO: I2C初期化
import smbus2
self.i2c_bus = smbus2.SMBus(1)  # I2C bus 1
```

#### `read_encoders()` メソッド
```python
# TODO: PR2040からエンコーダ値を読み取る
# レジスタアドレス:
#   REG_ENC0 = 0x10  # Left front
#   REG_ENC1 = 0x11  # Right front
#   REG_ENC2 = 0x12  # Left rear
#   REG_ENC3 = 0x13  # Right rear

import struct
left_front = self.read_i2c_register(0x10, 4)
left_rear = self.read_i2c_register(0x12, 4)
left_enc = (struct.unpack('<i', left_front)[0] +
           struct.unpack('<i', left_rear)[0]) // 2
# ... (right側も同様)
```

#### `read_i2c_register()` メソッド
```python
# TODO: I2Cレジスタ読み取り実装
self.i2c_bus.write_byte(self.i2c_address, register)
data = self.i2c_bus.read_i2c_block_data(self.i2c_address, register, num_bytes)
return bytes(data)
```

### 2. モーター制御ノード

**ファイル**: `mobile_robot_edge/motor_controller.py`

**実装箇所:**

#### `init_hardware()` メソッド
```python
# TODO: I2C初期化とモード設定
import smbus2
self.i2c_bus = smbus2.SMBus(1)

# 速度制御モードに設定
REG_SET_MODE_ALL = 0x06
mode_data = [REG_SET_MODE_ALL, 1, 1, 1, 1]  # All to velocity mode
self.i2c_bus.write_i2c_block_data(self.i2c_address, mode_data[0], mode_data[1:])
```

#### `set_wheel_velocities()` メソッド
```python
# TODO: モーター速度指令を送信
import struct

REG_SET_VEL_ALL = 0x08
velocities = struct.pack('<ffff',
                        left_cps,   # Wheel 0 (left front)
                        right_cps,  # Wheel 1 (right front)
                        left_cps,   # Wheel 2 (left rear)
                        right_cps)  # Wheel 3 (right rear)

data = [REG_SET_VEL_ALL] + list(velocities)
self.i2c_bus.write_i2c_block_data(self.i2c_address, data[0], data[1:])
```

### 3. カメラオートフォーカス設定

**ファイル**: `mobile_robot_edge/disable_camera_autofocus.sh`

起動前に実行して、フォーカス値を調整してください:

```bash
# フォーカス値の確認
v4l2-ctl -d /dev/video0 --list-ctrls | grep focus

# フォーカス値の調整（ロボットの動作距離に応じて）
# 近距離 (20-50cm): 200-350
# 中距離 (50cm-2m): 400-500
# 遠距離 (1m-infinity): 500-650
```

## パラメータ設定

`edge_bringup.launch.py`でパラメータを調整できます:

```python
parameters=[{
    'wheel_base': 0.16,              # 車輪間距離 (m)
    'wheel_radius': 0.033,           # 車輪半径 (m)
    'encoder_cpr': 720,              # エンコーダ分解能
    'max_linear_velocity': 0.5,      # 最大直進速度 (m/s)
    'max_angular_velocity': 2.0,     # 最大角速度 (rad/s)
    'i2c_address': 0x60,             # I2Cアドレス
}]
```

## トピック

### Published
- `/odom` (nav_msgs/Odometry) - オドメトリ情報
- `/camera/image_raw/compressed` (sensor_msgs/CompressedImage) - カメラ画像
- `/tf` (tf2_msgs/TFMessage) - 座標変換

### Subscribed
- `/cmd_vel` (geometry_msgs/Twist) - 速度指令

## TFツリー

```
odom
└── base_footprint
    └── base_link
        └── camera_link
            └── camera_optical_frame
```

## 依存パッケージ

```bash
sudo apt install ros-humble-v4l2-camera
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-robot-state-publisher
pip3 install smbus2  # I2C通信用
```

## トラブルシューティング

### カメラが起動しない
```bash
# カメラデバイスを確認
ls -l /dev/video*

# v4l2-ctlでカメラ情報確認
v4l2-ctl -d /dev/video0 --all
```

### I2C通信エラー
```bash
# I2Cデバイスを確認
i2cdetect -y 1

# 0x60にモータードライバが表示されるはず
```

### オドメトリが更新されない
- `init_hardware()`と`read_encoders()`のTODO実装を確認
- I2C接続とアドレスを確認

## ライセンス

MIT

## 作者

rx178nwj
