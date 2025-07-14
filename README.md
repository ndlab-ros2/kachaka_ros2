# kachaka_ros2

## はじめに

kachaka_ros2は、[カチャカ](https://kachaka.life/)を用いたROS 2ソフトウェア開発キットです。以下のパッケージ群から構成されています。

|パッケージ名|説明|
|---|---|
|kachaka_description|カチャカのdescriptionを記述するパッケージ。[kachaka_description](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/kachaka_description)から派生しています。|
|kachaka_interfaces|カチャカのアクション、メッセージファイルを管理するパッケージ。[kachaka_interfaces](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/kachaka_interfaces)から派生しています。|
|kachaka_nav2_bringup|Nav2 stackを起動するためのパッケージ。[kachaka_nav2_bringup](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/demos/kachaka_nav2_bringup)から派生しています。|
|kachaka_mapping|Mappingを起動するためのパッケージ|
|kachaka_gazebo|Gazebo Ignitionによるカチャカのシミュレーション環境を提供するパッケージ|
|utils/joy_controller|ユーティリティ|

kachaka_ros2_dev_kitは、カチャカ実機、シミュレーション環境で地図生成、自己位置推定、ナビゲーションを実行する機能を提供します。
シミュレーション環境（kachaka_gazeboパッケージ）を使ってナビゲーションを実行した例を下図に示します。

## 動作確認環境

以下の環境で動作確認しました。

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Fortress
- カチャカソフトウェア v3.8.5

`kachaka_gazebo`パッケージを使用する場合、シミュレーションやレンダリングの処理を高速にするため、NVIDIA GPU搭載環境での動作を推奨します。

## ビルド

```bash
sudo apt update
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/CyberAgentAILab/kachaka_ros2_dev_kit.git -b humble
cd ..
rosdep install -y -i --from-paths src
colcon build --symlink-install
source ~/dev_ws/install/setup.bash
```

## 使い方

### シミュレータ環境

- シミュレータ環境：[kachaka_gazebo/README.md](kachaka_gazebo/README.md)参照。

### 地図生成

- シミュレータ環境：[docs/sim/mapping_sim.md](docs/sim/mapping_sim.md)参照。
  - 通常、カチャカ実機を使う場合、カチャカ本体で地図生成も行われていることから、この機能はシミュレータ環境でのみ動作確認しています。

### ナビゲーション

- カチャカ実機：[docs/navigation.md](docs/navigation.md)参照。
- シミュレータ環境：[docs/sim/navigation_sim.md](docs/sim/navigation_sim.md)参照。

## ライセンス

|パッケージ名|ライセンス|
|---|---|
|kachaka_description|Apache License, Version 2.0|
|kachaka_interfaces|Apache License, Version 2.0|
|kachaka_nav2_bringup|Apache License, Version 2.0|
|kachaka_mapping|Apache License, Version 2.0|
|kachaka_gazebo|Apache License, Version 2.0|
|utils/joy_controller|MIT|
