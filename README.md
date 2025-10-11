# kachaka_ROS2_package

 [スマートファニチャー・プラットホーム「カチャカ」](https://kachaka.life/)ROS2用パッケージ群

## 目次
<!-- TOC -->

- [概要](#概要)
- [はじめに](#はじめに)
- [開発環境](#開発環境)
- [ハードウェア](#ハードウェア)
- [インストール方法](#インストール方法)
- [使用方法](#使用方法)

<!-- /TOC -->

## 概要
業務向け自律搬送ロボットの [カチャカ](https://kachaka.life/)を用いたNavigation2(Nav2)での自律移動やコントローラでの操作などのをROS2で制御する為のリポジトリを提供します。
主に以下のパッケージで構成されています

|パッケージ名|説明|
|---|---|
|kachaka_description|カチャカの各部分のリンクやジョイント、又はセンサの情報を表示するパッケージ。[kachaka_description](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/kachaka_description)に基づいています|
|kachaka_interfaces|カチャカのアクション、メッセージファイルを管理するパッケージ。[kachaka_interfaces](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/kachaka_interfaces)に基づいています|
|kachaka_nav2_bringup|Navigation2(Nav2)における自律走行を行うためのパッケージ。[kachaka_nav2_bringup](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/demos/kachaka_nav2_bringup)に基づいています|
|kachaka_gazebo|Gazebo Ignitionによるカチャカのシミュレーション環境を提供するパッケージ|
|joy_controller|お持ちのコントローラーでカチャカを操作する為のパッケージ|

こちらのリポジトリ(kachaka_ros2)では、カチャカの実機やシミュレーション開発を用いてナビゲーション等を実行する機能を提供します。
シミュレーション環境（kachaka_gazeboパッケージ）を使ってナビゲーションを実行した例を下図に示します。

|シミュレーション環境|シミュレーション環境上でのナビゲーション実行例|
|:---:|:---:|
|![](./docs/images/kachaka_gz_sim.png)|![](./docs/images/kachaka_gz_sim_navigation.png)|


## はじめに

先ずは、kachaka-apiを用いてカチャカをROS2につなげる必要があります

### カチャカAPIの有効化
> [!IMPORTANT]
> どんな使い方をする場合でも、まずはじめにスマートフォンアプリを使ってカチャカAPIを有効にする必要があります。

* カチャカに接続し、[⚙設定]のタブから接続するロボットを選択、[カチャカAPI]ページを開いて「カチャカAPIを有効化する」をONにします。
* ダイアログが表示されるので、「利用規約」を確認の上、「カチャカAPI利用規約に同意する」をチェックして「設定する」を押して下さい。

<table>
<tr>
<td><img src="./docs/images/spapp_kachaka_api_screen.png" width="150"></td>
<td><img src="./docs/images/spapp_kachaka_api_enable_dialog.png" width="150"></td>
</tr>
</table>

### カチャカのIPアドレスの確認
* またいずれの場合にも、カチャカのIPアドレスが必要になります。
* [⚙設定] > [アプリ情報] から確認することができます。(以下のキャプチャは白塗りしてあります)
* また、mDNSによる名前解決に対応しており、同画面の「シリアル番号」からなる
    * `kachaka-<シリアル番号>.local`というホスト名でもアクセス可能です。

<table>
<tr>
<td><img src="./docs/images/spapp_kachaka_app_info.png" width="150"></td>
<td><img src="./docs/images/spapp_kachaka_app_info_screen.png" width="150"></td>
</tr>
</table>

kachaka-apiのクローン作成
```bash
cd
git clone https://github.com/pf-robotics/kachaka-api.git
```

kachaka-apiによる接続
```bash
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <kachakaのIPアドレス>
```

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
