# ROS 2 Docker クイックスタート

## 基本コマンド

### コンテナに入る

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2
```

コンテナ内のシェルが起動します。ROS 2環境は自動的にセットアップされます。

### ROS 2コマンドの実行

コンテナ内で:

```bash
# トピックリスト表示
ros2 topic list

# ノードリスト表示
ros2 node list

# パッケージのビルド
cd /ros2_ws
colcon build --packages-select mobile_robot_edge
source install/setup.bash

# launchファイルの実行
ros2 launch mobile_robot_edge edge_bringup.launch.py
```

### ワンライナーでのコマンド実行

コンテナに入らずにコマンドを実行:

```bash
# ROS 2トピックリスト
sudo docker compose run --rm ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"

# パッケージビルド
sudo docker compose run --rm ros2 bash -c "cd /ros2_ws && colcon build --packages-select mobile_robot_edge"
```

## ハードウェアアクセス確認

### I2C (PR2040モータードライバ)

```bash
# コンテナ内で
i2cdetect -y 1
# 0x60にPR2040が表示されるはず
```

### カメラ

```bash
# コンテナ内で
v4l2-ctl -d /dev/video0 --list-formats-ext
```

## エッジサービスとの連携

### Edge Servicesの起動 (別ターミナル)

```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

これにより以下のサービスが起動:
- モーター制御: `ws://localhost:8001`
- オドメトリ: `ws://localhost:8002`
- カメラ: `ws://localhost:8003`

### ROS 2からの接続

Edge ServicesとROS 2を連携するには、ROS 2ブリッジノードを作成します。

例: WebSocketからROS 2トピックへのブリッジ

```python
# /ros2_ws/src/mobile_robot_edge/mobile_robot_edge/ws_bridge.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import asyncio
import websockets
import json

class WebSocketBridge(Node):
    def __init__(self):
        super().__init__('ws_bridge')

        # ROS 2 subscribers/publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # WebSocket接続
        self.motor_ws_uri = "ws://localhost:8001"
        self.odom_ws_uri = "ws://localhost:8002"

    async def cmd_vel_callback(self, msg):
        # cmd_velをWebSocketで送信
        async with websockets.connect(self.motor_ws_uri) as ws:
            cmd = {
                "type": "cmd_vel",
                "linear": msg.linear.x,
                "angular": msg.angular.z
            }
            await ws.send(json.dumps(cmd))

    # ... オドメトリ受信の実装 ...
```

## よく使うコマンド

### イメージの再ビルド

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose build --no-cache
```

### コンテナのクリーンアップ

```bash
# 停止中のコンテナを削除
sudo docker compose down

# 未使用のイメージを削除
sudo docker image prune -a
```

### ログの確認

```bash
# コンテナのログ
sudo docker compose logs

# ROS 2のログ (コンテナ内)
ros2 run rqt_console rqt_console
```

## トラブルシューティング

### "permission denied" エラー

```bash
# dockerグループに所属しているか確認
groups

# ログアウト/ログインまたは
newgrp docker
```

### I2Cデバイスが見えない

```bash
# ホスト側で権限確認
ls -l /dev/i2c-1
sudo chmod 666 /dev/i2c-1
```

### カメラが使えない

```bash
# ホスト側で権限確認
ls -l /dev/video0
sudo chmod 666 /dev/video0
```

## 次のステップ

1. **WebSocketブリッジの実装**: ROS 2とEdge Servicesを接続
2. **Navigation Stackの統合**: Nav2パッケージの追加とセットアップ
3. **SLAMの実装**: SLAM Toolboxの設定
4. **自律ナビゲーション**: ゴール指定での自律移動の実装
