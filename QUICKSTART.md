# Mobile Robot - クイックスタート

## システム起動

### 1. edge_servicesの起動

ターミナル1:

```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

正常に起動すると:
```
WARNING:hardware.pr2040_driver:PR2040 Driver not responding - running in dummy mode
```

ポート8001, 8002が起動します:
```bash
netstat -tuln | grep -E ":(8001|8002)"
```

### 2. ROS 2ノードの起動（Docker環境）

ターミナル2:

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2
```

コンテナ内で:

```bash
source /ros2_ws/install/setup.bash
ros2 launch mobile_robot_edge ws_edge_bringup.launch.py
```

### 3. 動作確認

ターミナル3:

```bash
cd /home/pi/MobileRobot/ros2_docker
sudo docker compose run --rm ros2
```

コンテナ内で:

```bash
source /ros2_ws/install/setup.bash

# トピックリスト表示
ros2 topic list

# オドメトリ表示
ros2 topic echo /odom

# モーター制御テスト
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
```

## 簡易起動スクリプト

### edge_servicesの起動

```bash
cd /home/pi/MobileRobot/edge_services
python3 launch_all.py
```

### ROS 2の起動

```bash
cd /home/pi/MobileRobot/ros2_docker
./run_edge_bringup.sh
```

## トラブルシューティング

### edge_servicesが起動しない

依存パッケージをインストール:

```bash
cd /home/pi/MobileRobot/edge_services
pip3 install --break-system-packages -r requirements.txt
```

### Dockerコンテナに入れない

Docker権限を確認:

```bash
sudo usermod -aG docker pi
newgrp docker
```

### ROS 2パッケージが見つからない

コンテナ内でビルド:

```bash
cd /ros2_ws
colcon build --packages-select mobile_robot_edge
source install/setup.bash
```

## 詳細ドキュメント

- [DOCKER_SETUP_SUMMARY.md](DOCKER_SETUP_SUMMARY.md) - 完全なセットアップガイド
- [ros2_docker/README.md](ros2_docker/README.md) - Docker環境詳細
- [mobile_robot_edge/README_DOCKER.md](mobile_robot_edge/README_DOCKER.md) - パッケージ使用方法
- [edge_services/README.md](edge_services/README.md) - Edge Services詳細
