# gimbalrotor 制御システム概要とbringup時トピック一覧

調査対象: `aerial_robot_control` パッケージ、`gimbalrotor` パッケージ（`robots/gimbalrotor/launch/bringup.launch`）

## 1. 全体アーキテクチャ

`gimbalrotor/launch/bringup.launch` が起動する主な要素:

1. **`aerial_robot_base_node`**（`aerial_robot_base` パッケージ）— 中核ノード。以下を保持する:
   - `RobotModelRos`（URDF由来の機体モデル）
   - `StateEstimator`（IMU/GPS/mocap等を融合した状態推定、EGOMOTION/EXPERIMENT/GROUND_TRUTHの3モード）
   - `pluginlib` でロードされる **Navigator**（`flight_navigation_plugin_name = aerial_robot_navigation/gimbalrotor_navigation` → `GimbalrotorNavigator`、`gimbalrotor/gimbalrotor_navigation.h/.cpp`）
   - `pluginlib` でロードされる **Controller**（`aerial_robot_control_name = aerial_robot_control/gimbalrotor_controller` → `GimbalrotorController`、`GimbalrotorControl.yaml` で指定、`gimbalrotor/control/gimbalrotor_controller.h/.cpp`）
   - `main_rate`（40Hz）の専用タイマスレッドで `navigator_->update(); controller_->update();` を毎周期実行（`aerial_robot_base/src/aerial_robot_base.cpp`）

2. **`aerial_robot_model.launch`** — robot_state_publisher / rviz 表示用

3. **`sensors.launch.xml`** — 実機なら `spinal` のシリアルブリッジ（IMU/GPSがFCから来る）とmocap、シミュレーションならGazebo側センサ

4. **`servo_bridge_node`**（`aerial_robot_model`）— ジンバル関節コマンドを実サーボ指令に変換

5. **Gazebo**（`simulation:=true` 時）

### 制御の流れ

Navigator が飛行状態（ARM_OFF→START→ARM_ON→TAKEOFF→HOVER→LAND→STOP）と目標位置/速度/加速度/姿勢を管理
→ Controller（`GimbalrotorController` は `PoseLinearController`/fully-actuated PIDベースを継承）が各軸PID誤差からCOG系の目標wrench(力・トルク)を計算
→ ジンバル込みで疑似逆行列によるロータ推力配分（`integrated_map_inv`）を解き、各ロータの推力とジンバル角を算出
→ `spinal`（フライトコントローラFW）へ司令を送信。

この機体（beatle等のgimbalrotor）は各ロータにジンバル（`Servo.yaml`で定義）を持つ全アクチュエータ型（fully-actuated、`underactuate: false`がデフォルト）で、推力方向とは独立に機体姿勢を制御できるのが最大の特徴。そのため `desire_coordinate`（姿勢指令）と `uav/nav`（並進運動指令）が分離されており、`torque_allocation_matrix_inv` を介して実際のロータ推力・ジンバル角配分計算をspinal（フライトコントローラFW）側にオフロードする設計になっている（`gimbal_calc_in_fc: true` がbeatleのデフォルト）。

## 2. bringup時の主要トピック（`/gimbalrotor<id>/...` 名前空間）

### 2.1 状態推定（StateEstimator, `aerial_robot_estimation`）

| トピック | 型 | 内容 |
|---|---|---|
| `uav/cog/odom` | `nav_msgs/Odometry` | 機体重心(COG)の姿勢・速度推定値 |
| `uav/baselink/odom` | `nav_msgs/Odometry` | ベースリンクの姿勢・速度推定値 |
| `uav/full_state` | `aerial_robot_msgs/States` | 各カルマンフィルタ内部状態の全ダンプ(デバッグ用) |
| `imu1/ros_converted` | `sensor_msgs/Imu` | spinalの生IMUをROS標準形式に変換したもの |
| `imu1/acc_only` | `aerial_robot_msgs/Acc` | 加速度のみ抽出 |
| `imu1/filter_angular_velocity` | `geometry_msgs/Vector3Stamped` | フィルタ後角速度 |
| `gps1/ros_converted` | `sensor_msgs/NavSatFix` | GPS変換値 |

### 2.2 Navigator（`BaseNavigator` + `GimbalrotorNavigator`）

**Subscribe（入力・司令）**

| トピック | 型 | 内容 |
|---|---|---|
| `uav/nav` | `aerial_robot_msgs/FlightNav` | メインの航法指令。x/y/z毎にPOS/VEL/ACC/POS_VELモード、roll/pitch/yawの目標値・角速度 |
| `target_pose` | `geometry_msgs/PoseStamped` | 単一ウェイポイント目標 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | rvizからのゴール指定 |
| `target_path` | `nav_msgs/Path` | 経路/軌道 |
| `simple_nav` | `aerial_robot_msgs/SimpleFlightNav` | 簡易navコマンド(x/y/z各軸のPOS/VEL/ACCモード切替) |
| `teleop/takeoff`, `land`, `start`, `halt`, `force_landing` | `std_msgs/Empty` | 離陸/着陸/アーム/緊急停止/強制着陸コマンド |
| `teleop/x_ctrl_mode` 等(x/y/z/body_x/y/z) | `std_msgs/Int8` | 各軸の制御モード(0:位置,1:速度,2:加速度)切替 |
| `joy` | `sensor_msgs/Joy` | ジョイスティック操縦 |
| `battery_voltage_status` | `std_msgs/Float32` | バッテリ電圧(低電圧判定用) |
| `flight_config_ack` | `std_msgs/UInt8` | spinalからのARM ON/OFF・強制着陸ACK |
| `final_target_baselink_rot`/`final_target_baselink_rpy` | `geometry_msgs/QuaternionStamped`/`Vector3Stamped` | 機体姿勢(ベースリンク)の目標(推力方向と独立に姿勢指定できるのがgimbalrotorの特徴) |

**Publish（出力）**

| トピック | 型 | 内容 |
|---|---|---|
| `flight_config_cmd` | `spinal/FlightConfigCmd` | ARM/DISARM/強制着陸コマンドをspinalへ送信 |
| `flight_state` | `std_msgs/UInt8` | 現在の飛行状態(ARM_OFF/START/ARM_ON/TAKEOFF/HOVER/LAND/STOP) |
| `desire_coordinate` | `spinal/DesireCoord` | 目標ベースリンクroll/pitch/yaw(spinal行き) |
| `trajectory` | `nav_msgs/Path` | 生成された軌道(可視化用) |
| `waypoints` | `visualization_msgs/MarkerArray` | ウェイポイント可視化 |
| `uav_power` | `geometry_msgs/Vector3Stamped` | 電源/バッテリ情報 |
| `x_control_mode`等 | `std_msgs/UInt8` | 現在の各軸制御モードのエコー |

### 2.3 Controller（`GimbalrotorController`）

**Publish**

| トピック | 型 | 内容 |
|---|---|---|
| `four_axes/command` | `spinal/FourAxisCommand` | spinalへの最終指令。`angles[3]`=roll,pitch,yaw項 + `base_thrust[]`=各仮想ロータの推力配列 |
| `torque_allocation_matrix_inv` | `spinal/TorqueAllocationMatrixInv` | 推力→トルク変換の逆行列(1000倍スケール)。`gimbal_calc_in_fc: true`(beatleのデフォルト)時、ジンバル角込みの配分計算をspinal側で行うために送信 |
| `gimbal_dof` | `std_msgs/UInt8` | ロータあたりのジンバル自由度(1 or 2) |
| `rpy/gain` | `spinal/RollPitchYawTerms` | roll/pitch/yawのPIDゲイン(spinal行き、1000倍スケール) |
| `motor_info`/`uav_info` | `spinal/PwmInfo`/`spinal/UavInfo` | モータ較正情報・機体モデル情報(アーム時に送信) |
| `debug/pose/pid`, `debug/pose/pid_body` | `aerial_robot_msgs/PoseControlPid` | x/y/z/roll/pitch/yaw各PID項(P/I/D, 目標値, 誤差)の詳細デバッグ |
| `debug/pose/feedforward_term(_body)` | `geometry_msgs/Twist` | フィードフォワード項 |
| `feedforward_acc_world`, `feedforward_ang_acc_cog` | `geometry_msgs/Vector3Stamped` | 外力補償用のフィードフォワード並進/回転加速度 |
| `wrench_error_cog` | `geometry_msgs/WrenchStamped` | 目標wrenchと推定外力の誤差(COG系) |
| `estimated_external_wrench` | `geometry_msgs/WrenchStamped` | モーメンタムオブザーバによる推定外力(生値) |
| `filtered_est_external_wrench` | `geometry_msgs/WrenchStamped` | 上記のLPF後の値(接触作業などで使用) |
| `gimbals_ctrl`/`debug/target_vectoring_force` | `sensor_msgs/JointState`/`std_msgs/Float32MultiArray` | `gimbal_calc_in_fc: false`時のみ使用(beatleでは基本未使用) |

**Subscribe**

| トピック | 型 | 内容 |
|---|---|---|
| `desire_wrench` | `geometry_msgs/WrenchStamped` | 外部から与える目標wrench(接触作業等)。COG系に変換して制御に反映 |
| `attaching_flag` | `std_msgs/Bool` | ドッキング/接触中フラグ |
| `xyz_wrench_control_flag` | `std_msgs/Bool` | XYZ直接wrench制御モードの切替 |
| `desire_pos_for_impedance` | `geometry_msgs/Vector3` | インピーダンス制御の目標位置(body-x方向) |

## 3. 参照した主なソースファイル

- `robots/gimbalrotor/launch/bringup.launch`
- `robots/gimbalrotor/launch/include/sensors.launch.xml`
- `robots/gimbalrotor/include/gimbalrotor/control/gimbalrotor_controller.h`
- `robots/gimbalrotor/src/control/gimbalrotor_controller.cpp`
- `robots/gimbalrotor/include/gimbalrotor/gimbalrotor_navigation.h`
- `robots/gimbalrotor/src/gimbalrotor_navigation.cpp`
- `robots/gimbalrotor/config/beatle/GimbalrotorControl.yaml`
- `aerial_robot_control/include/aerial_robot_control/control/base/base.h`
- `aerial_robot_control/include/aerial_robot_control/flight_navigation.h`
- `aerial_robot_control/src/flight_navigation.cpp`
- `aerial_robot_control/src/control/base/pose_linear_controller.cpp`
- `aerial_robot_base/src/aerial_robot_base.cpp`
- `aerial_robot_base/include/aerial_robot_base/aerial_robot_base.h`
- `aerial_robot_base/src/aerial_robot_base_node.cpp`
- `aerial_robot_estimation/src/state_estimation.cpp`
- `aerial_robot_estimation/src/sensor/imu.cpp`, `gps.cpp`
- `aerial_robot_msgs/msg/{PoseControlPid,FlightNav,SimpleFlightNav,States}.msg`
- `aerial_robot_nerve/spinal/msg/{FourAxisCommand,DesireCoord,TorqueAllocationMatrixInv}.msg`
