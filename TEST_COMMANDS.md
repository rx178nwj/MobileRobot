# 動作確認コマンド

## 前提条件

### ターミナル1: edge_services起動中
```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

### ターミナル2: ROS 2ノード起動中
```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2 bash -c "source /ros2_ws/install/setup.bash && ros2 launch mobile_robot_edge ws_edge_bringup.launch.py"
```

## 動作確認（ターミナル3）

### Dockerコンテナに入る

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2
```

### コンテナ内で実行

```bash
# 環境設定
source /ros2_ws/install/setup.bash

# 1. トピックリスト確認
ros2 topic list
# 期待される出力:
# /cmd_vel
# /odom
# /parameter_events
# /robot_description
# /rosout
# /tf
# /tf_static

# 2. ノードリスト確認
ros2 node list
# 期待される出力:
# /robot_state_publisher
# /ws_motor_controller
# /ws_odometry_publisher

# 3. オドメトリデータ確認
ros2 topic echo /odom --once
# オドメトリメッセージが表示される

# 4. TF確認
ros2 topic echo /tf --once
# odom -> base_footprint のTFが表示される

# 5. モーター制御テスト - 前進
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 6. モーター制御テスト - 回転
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once

# 7. モーター制御テスト - 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 8. トピック周波数確認
ros2 topic hz /odom
# 約50Hzで配信されているはず

# 9. TFツリー確認（PDF生成）
ros2 run tf2_tools view_frames
# frames.pdfが生成される（グラフィカル環境が必要）

# 10. rqt_graph でノードグラフ確認（グラフィカル環境が必要）
# rqt_graph
```

## edge_servicesログ確認

別のターミナルで:

```bash
# edge_servicesのログを確認
tail -f /home/pi/MobileRobot/edge_services/logs/edge_services.log

# または、実行中のプロセスの出力を確認
ps aux | grep launch_all
```

## WebSocket接続テスト（Pythonスクリプト）

```python
# test_websocket.py
import asyncio
import websockets
import json

async def test_motor():
    uri = "ws://localhost:8001"
    async with websockets.connect(uri) as ws:
        cmd = {"type": "cmd_vel", "linear": 0.1, "angular": 0.0}
        await ws.send(json.dumps(cmd))
        response = await ws.recv()
        print(f"Motor response: {response}")

async def test_odom():
    uri = "ws://localhost:8002"
    async with websockets.connect(uri) as ws:
        for i in range(5):
            msg = await ws.recv()
            data = json.loads(msg)
            print(f"Odom {i}: x={data['pose']['x']:.3f}, y={data['pose']['y']:.3f}")

# 実行
asyncio.run(test_motor())
asyncio.run(test_odom())
```

```bash
# 実行
python3 test_websocket.py
```

## トラブルシューティング

### WebSocket接続失敗

```bash
# edge_servicesが起動しているか確認
netstat -tuln | grep -E ":(8001|8002)"

# プロセス確認
ps aux | grep launch_all

# edge_services再起動
pkill -f launch_all.py
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

### ROS 2ノードが起動しない

```bash
# パッケージ再ビルド
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2 bash -c "cd /ros2_ws && colcon build --packages-select mobile_robot_edge"
```

### オドメトリが配信されない

```bash
# ws_odometry_publisherのログレベルをdebugに
ros2 run mobile_robot_edge ws_odometry_publisher --ros-args --log-level debug
```

## 成功の確認ポイント

✅ edge_servicesが起動し、ポート8001, 8002がLISTEN状態
✅ ROS 2ノードが3つ起動（ws_motor_controller, ws_odometry_publisher, robot_state_publisher）
✅ WebSocket接続が"connected"と表示
✅ `ros2 topic list`で/cmd_vel, /odom, /tfが表示される
✅ `ros2 topic echo /odom`でオドメトリデータが表示される
✅ `ros2 topic pub /cmd_vel`でモーター制御コマンドが送信できる
