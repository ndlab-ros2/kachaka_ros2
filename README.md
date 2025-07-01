# kachaka_ros2

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
