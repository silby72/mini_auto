# mini_auto

slam_ros2 (LittleSLAM の ROS 2 ラッパ) で地図を作りながら、室内を自律移動するロボット。

- 足回り: 4輪オムニ (ros2can の `serial_tx_[ID]` に duty を送る)
- オドメトリ: 三角形に配置した3輪の計測輪 (ros2can の `serial_rx_[ID]_zeroed`) で並進、WT901C (wit_node の `/imu`) で回転
- SLAM: [slam_ros2](https://github.com/silby72/slam_ros2) (サブモジュール `external/slam_ros2`)
- ナビ: 自作の A* + pure pursuit (ホロノミック)。RViz の 2D Goal Pose で目標を与える

```
 TG30 ──/scan─────────┬──────────────────────► slam_node ──TF map→odom, /map──┐
                      │                            ▲                          │
 D456 ─depth image─► depth_obstacle ─/depth_scan─┐ │ TF odom→base_link        │
                      │                          │ │                          ▼
 WT901C ─/imu──────► tracking_odom ──────────────┼─┘                      simple_nav ◄─ /goal_pose (RViz)
 ros2can ─serial_rx_N_zeroed─┘                   └────────(障害物)──────────►  │
    ▲                                                                         │
    └──serial_tx_N── omni_drive ◄──────────────────/cmd_vel───────────────────┘
```

## ノード

| ノード | 役割 |
|:--|:--|
| `omni_drive` | `cmd_vel` → 加速度制限 → 4輪の転がり速度 → duty。1輪でも上限を超えたら全輪を同率で落とす |
| `tracking_odom` | 3輪の移動量から (dx, dy, dθ) を求めて積分。`odom` と TF `odom→base_link`。`use_imu: true` なら dθ は IMU の yaw 差分、(dx, dy) は回転ぶんを引いた最小二乗。IMU が途切れたら計測輪のみに戻る |
| `depth_obstacle` | D456 の深度画像を base_link 座標の点に戻し、床より上・機体の高さ以下の点だけを全周 LaserScan (`/depth_scan`) にする |
| `simple_nav` | `/map` + 障害物スキャン (`/scan`, `/depth_scan`, 数秒間記憶) を膨張させて A*、先読み点へ向かう速度ベクトルを出す。進行方向の障害物で減速・停止 |
| `sim_robot` | 実機なし用。ros2can・機体・TG30・WT901C・D456 の代わり。`serial_tx` を受けて動き、カウント・スキャン・IMU・深度画像を返す |

車輪はすべて「中心からの位置 (x, y)」と「正転時に接地点が進む向き dir」で表すので、
配置が変わっても `config/robot.yaml` を書き換えるだけでよい。

## ビルド

```sh
git clone --recurse-submodules https://github.com/silby72/mini_auto.git
cd mini_auto
colcon build
source install/setup.bash
```

`src/slam_ros2` は `external/slam_ros2/slam_ros2` へのシンボリックリンク。
`external/` には `COLCON_IGNORE` を置いて、upstream の素の CMake を colcon が拾わないようにしている。

## シミュレーション

```sh
ros2 launch mini_auto sim.launch.py
```

RViz の「2D Goal Pose」でゴールを置く。`/sim/ground_truth`(真値の座標軸)と TF `base_link` のずれが SLAM の誤差。
壁の配置は `config/sim.yaml` の `walls`。

## 実機

1. ros2can を起動し、計測輪エンコーダの原点セットまで済ませる (`serial_rx_[ID]_zeroed` が出る状態)
2. `config/robot.yaml` の【仮置き】をすべて実機に合わせる
3. 起動

```sh
ros2 launch mini_auto robot.launch.py laser_x:=0.0 laser_y:=0.0 laser_yaw:=0.0
```

### 立ち上げ時の確認手順

1. **IMU**: WT901C を出力 100 Hz・6軸モード(屋内は磁気が乱れるので使わない)に設定。水平に置き、機体を反時計回りに回して `/imu` の yaw が増えるか(減るなら `imu_yaw_sign: -1`)
1. **D456**: `rs-enumerate-devices` で USB 3.x 接続か確認。`cam_*` の取り付け値を合わせ、何もない床で `/depth_scan` が空になるか(床を拾うなら `cam_pitch` を見直すか `min_height` を上げる)
1. **計測輪の向き**: 機体を手で前 (+x) に押して `ros2 topic echo /odom` の x が増えるか。左 (+y)、反時計回り (+yaw) も同様。合わなければ `tracking_signs`
2. **計測輪のスケール**: 1 m 押して 1 m になるか → `tracking_wheel_radius` / `tracking_counts_per_rev`。その場で 1 回転させて yaw が 2π 戻るか → 取り付け位置
3. **駆動輪の向き**: 浮かせて `ros2 topic pub /cmd_vel ...` で vx, vy, wz を1つずつ与え、各輪の回転方向を確認 → `drive_signs`
4. **teleop で地図作り**: `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (`b` でホロノミックモード) で一周させ、地図が崩れないか
5. **ナビ**: 低速 (`max_vel: 0.2`) から 2D Goal Pose を試す

## 注意

- slam_ros2 の地図は「ヒットグリッド」(占有=100、未知=-1)で空きセルの情報がない。ナビは未知を通行可として扱うため、
  まだ見ていない場所へのゴールも「通れる前提」で計画し、近づいてスキャンに映った障害物で再計画する
- ナビの停止判定・減速は機体を円 (`robot_radius`) とみなしている
- D456 は前方しか見えず、約 0.4 m より近くは測れない。横移動で視野外の低い障害物に寄る場合は記憶 (`obstacle_memory`) 頼み
- ドライブはオープンループ。位置のずれは SLAM の自己位置で外側から閉じている
