# humbleble_ws
実機やシミュレーションを動かすときに必要なパッケージたちです。
シミュレーションは実装途中なので、おいおいやっていきます。
またmoveit2を動かすときに、urdfが読み込めなくて実行できないときがあるかもしれません。

## インストール！！

1. リポジトリをクローン
```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws/src

git clone https://github.com/danbo-rusenki/humbleble_ws.git -b sim-ign
git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

```

2. ワークスペースをビルド
```bash

sudo apt update && sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-velocity-controllers ros-humble-effort-controllers ros-humble-joint-trajectory-controller ros-humble-position-controllers ros-humble-robot-state-publisher ros-humble-xacro

cd ~/ros2_humble_ws
colcon build --symlink-install
```
3. pythonファイルに実行権限を付与、ファイルのあるディレクトリに移動するか、ファイルの場所を指定してください。
```bash
chmod +x rover_twist_relay.py

chmod +x joint_state_filter.py
```

## コマンド

1. gazebo 立ち上げ
```bash
source install/setup.bash 
ros2 launch amir_gazebo gazebo_bringup.launch.py
```
　箱を出現させる
```bash
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_box.launch.py 
```
```bash
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_multibox.launch.py 
```
　壁を出現させる
```bash
source install/setup.bash 
ros2 launch mecanumrover3_gazebo spawn_wor.launch.py scale:=0.001
```
2. 初期位置移動
```bash
source install/setup.bash 
ros2 run amir_operation initial_posi_gz
```

3. moveit2 
```bash
source install/setup.bash 
ros2 launch amir_moveit_config moveit_gazebo.launch.py 
```

4. moveit2に指示送る
```bash
source install/setup.bash 
ros2 launch amir_operation pick_place_fix_launch.py 
```

5. nav2 
```bash
source install/setup.bash 
ros2 launch mecanum_navigation2 bringup_launch.py
```

