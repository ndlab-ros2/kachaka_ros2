# kachaka_ros2

## はじめに

```bash
mkdir -p ~/your_ws/src
cd ~/your_ws/src
git clone https://github.com/CyberAgentAILab/kachaka_ros2_dev_kit.git
cd ..
rosdep install -y -i --from-paths src
colcon build --packages-skip wiimote
source install/setup.bash
```
