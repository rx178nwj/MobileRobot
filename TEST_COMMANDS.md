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

---

## SLAM 2.5D 受け入れ確認（実機）

### 前提

- LiderModule: `/dev/lider_module`
- PR2040: `/dev/pr2040`
- edge_services 起動済み
- ROS 2 workspace を build 済み

### 1) Edge 側 bringup

```bash
ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
```

### 2) Server 側 bringup

```bash
ros2 launch mobile_robot_server server_bringup.launch.py
```

### 3) `/scan` 要件確認（F-02）

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

判定:
- 周波数が 5 Hz 以上
- `header.frame_id: laser_frame`
- `range_min: 0.02`、`range_max: 12.0`

### 4) `/lider/pointcloud2` 要件確認（F-03）

```bash
ros2 topic hz /lider/pointcloud2
ros2 topic echo /lider/pointcloud2 --once
```

判定:
- `pointcloud_interval_s:=2.0` 相当の周期
- `header.frame_id: laser_frame`
- `fields` に `x,y,z`

### 5) TF tree 要件確認（F-05）

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map laser_frame
```

判定:
- `map -> odom -> base_footprint -> base_link -> laser_frame -> imu_link` が成立

### 6) SLAM 要件確認（F-06）

```bash
ros2 topic echo /map --once
```

判定:
- `/map` が配信される
- `map -> odom` TF が解決できる

### 7) Nav2 local voxel layer 確認

```bash
ros2 param get /local_costmap/local_costmap plugins
```

判定:
- `["obstacle_layer", "voxel_layer", "inflation_layer"]` を含む

---

## SLAM 2.5D 正式再測定手順（再現性重視）

低頻度トピック（`/lider/pointcloud2`）の取り逃しを防ぐため、**同じ60秒窓で同時観測**する。

### A) 計測プロファイル（検証用）

- `pointcloud_interval_s:=2.0`
- `scan_priority_cycles_before_3d:=4`
- `RMW_FASTRTPS_USE_SHM=0`（DDS SHM競合回避）

### B) 起動（ターミナル1）

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 launch mobile_robot_edge ws_edge_bringup.launch.py pointcloud_interval_s:=2.0 scan_priority_cycles_before_3d:=4
```

### C) 起動（ターミナル2）

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 launch mobile_robot_server server_bringup.launch.py
```

### D) 同時60秒観測（ターミナル3）

```bash
source /ros2_ws/install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

(timeout 60 ros2 topic echo /rosout > /tmp/rosout_60s.log 2>&1) &
(timeout 60 ros2 topic echo /scan > /tmp/scan_60s.log 2>&1) &
(timeout 60 ros2 topic echo /lider/pointcloud2 > /tmp/pc2_60s.log 2>&1) &
wait

grep -c "^header:" /tmp/scan_60s.log
grep -c "^header:" /tmp/pc2_60s.log
grep -c "2D slice timeout" /tmp/rosout_60s.log
grep -c "scan complete with no slice after grace wait" /tmp/rosout_60s.log
grep -c "3D continuous sweep start" /tmp/rosout_60s.log
grep -c "PointCloud2:" /tmp/rosout_60s.log
```

### E) 判定基準（正式）

- `/scan` は **60秒で1件以上**（`grep -c "^header:" /tmp/scan_60s.log`）
- `/lider/pointcloud2` は **60秒で1件以上**（`grep -c "^header:" /tmp/pc2_60s.log`）
- warning:
  - `2D slice timeout` は 0 件を目標（1件は単発遅延として記録）
  - `scan complete with no slice after grace wait` は 0 件
- 3D動作ログ:
  - `3D continuous sweep start` が1回以上
  - `PointCloud2:` が1回以上

### F) 備考

- 低頻度トピックは `ros2 topic hz` 単独判定だと取り逃しがあるため、**件数判定を正**とする。
- 本番運用時は必要に応じて既定値（例: `pointcloud_interval_s:=6.0`, `scan_priority_cycles_before_3d:=15`）へ戻す。
