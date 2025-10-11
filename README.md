# kachaka_ROS2_package

kachaka用ROS2パッケージ



## はじめに

ビルド
```bash
mkdir -p ~/your_ws/src
cd ~/your_ws/src
git clone https://github.com/ndlab-ros2/kachaka_ros2.git
cd ..
rosdep install -y -i --from-paths src
colcon build --packages-skip wiimote
source install/setup.bash
```

kachaka-apiによる接続
```bash
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <kachakaのIPアドレス>
```

※kachakaをアップデートした際は新しいDocker環境が必要になるので、以下のコマンドを実行して既存のDocker環境を削除しなければならない
```bash
# 停止中コンテナの削除（必要に応じて）
docker container prune -f

# 既存のros2_bridgeサービスの削除
docker-compose down --volumes --remove-orphans

# 古いイメージの削除（ピンポイントで削除するのが安全）
docker rmi asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:20250213
```
そして再びkachaka-apiでの接続を行う
```bash
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <kachakaのIPアドレス>
```

マップサーバーの起動
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/your/map.yaml
```
