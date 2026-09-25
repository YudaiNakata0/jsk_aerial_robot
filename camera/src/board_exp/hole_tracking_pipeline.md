# 穴トラッキング〜位置補償パイプライン

`hole_target_tracker.py` で壁の穴を画像上で追跡し、その偏差を打ち消すように
gimbalrotor (Beatle) の y/z を制御する一連の処理の流れと使い方をまとめる。

## 1. 全体の流れ

```
 /usb_cam/image_raw
        │
        ▼
 board_exp/hole_target_tracker.py        (または feature_target_tracker.py : 同じトピックを出す)
        │  /target/hole_deviation      Pose2D  基準点からの穴の画素偏差 (x=du, y=dv)
        │  /target/hole_tracking_valid Bool    検出できているか
        │  /target/2D_position         Vector3 穴の画素座標 (z=マッチスコア)
        ▼
 ┌──────────────── 後段 (どれか1つだけ起動する) ────────────────┐
 │ robot_control/hole_deviation_compensator.py                   │ 画素PID + ロスト時は検出できていた位置へ戻る
 │ robot_control/hole_world_estimate_compensator.py              │ 画素PID + 穴を合わせられる機体位置p*を推定し、ロスト時はp*へ戻る
 │ robot_control/hole_pixel_scale_calibration.py (事前に1回)     │ p*推定用の換算係数[m/px]を求めるキャリブレーション飛行
 └───────────────────────────────────────────────────────────────┘
        │  /gimbalrotor/simple_nav  SimpleFlightNav (y/z 速度指令、キャリブレーション時のみ位置指令)
        ▼
 GimbalrotorNavigator (HOVER中のみ有効)
```

- 後段ノードはどれも `/gimbalrotor/simple_nav` に送信するため、**同時に2つ以上起動しないこと**。
- 位置の記録・復帰には `/gimbalrotor/uav/cog/odom` (world系) の y, z を使う。
- `simple_nav` は `HOVER` 状態でしか受け付けられないため、離陸・ホバリング後に使う。

## 2. hole_target_tracker.py

穴そのものではなく、**穴の下側の壁面**(ロボットの映り込みが少なく壁の模様を含む範囲)を
広めのテンプレートとして切り出し、テンプレートマッチングでその移動を追跡する。
穴の位置はテンプレートからの相対オフセットで逆算する。

### 状態とキー操作 (OpenCVウィンドウ上で操作)

| 状態 | 操作 | 内容 |
|---|---|---|
| capture | `s` | ライブ映像の1フレームをサンプル画像として固定 |
| select | クリック | サンプル画像上で穴の中心をクリック → テンプレート切り出し、tracking へ |
| tracking | `c` → クリック | ライブ映像上のクリック位置を新しい**基準点**にする(画面を止める必要なし) |
| 任意 | `r` | リセットして capture からやり直し |

- 基準点の初期値は穴をクリックした位置。偏差は `穴の現在位置 − 基準点` [px]。
- 基準点は「穴をこの画素位置に持ってきたい」という目標。エンドエフェクタの位置などに合わせて `c` で設定する。

### 探索

- 通常は前回位置の周囲 `search_margin` だけを探索する(窓探索)。
- `lost_frames_before_full_search` フレーム連続で見失ったら全画面探索。`use_coarse_reacquire` なら縮小画像で粗く探索してから原寸で詰める。
- スコア (`TM_CCOEFF_NORMED`) が `match_thresh` 未満なら無効。無効な間は `/target/hole_deviation` は配信されず、`/target/hole_tracking_valid` に false が出る。

### 主なパラメータ

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `~compressed` | false | true で `/usb_cam/image_raw/compressed` を購読 |
| `~topic` | 上記 | 入力画像トピック |
| `~patch_width` / `~patch_height` | 480 / 300 | テンプレートの大きさ [px] |
| `~patch_top_margin` | 40 | 穴の中心からテンプレート上端までの距離 [px] |
| `~match_thresh` | 0.7 | 有効判定のスコア閾値 |
| `~search_margin` | 60 | 窓探索の余白 [px] |
| `~lost_frames_before_full_search` | 5 | 全画面探索に切り替えるまでのロストフレーム数 |
| `~use_coarse_reacquire` / `~coarse_scale` | true / 0.5 | 全画面探索の粗密探索 |

デバッグ画像は `/processed_image/hole_tracking` (テンプレート枠: 緑=有効/赤=無効、赤丸=穴、青十字=基準点)。

### 代替: feature_target_tracker.py

特徴点マッチング(AKAZE/SIFT/ORB + RANSAC)版。出力トピックが同じなので、
テンプレートマッチングで見失いやすい場合はそのまま差し替えられる。

## 3. 後段ノード

### 3.1 hole_deviation_compensator.py (シンプル版)

画素偏差をそのまま誤差として y/z の速度をPIDで出す。画像の右/下が正、機体の y/z とは逆向きなので負号を付けている。

**ロスト時の動作**

1. **HOLD**: `hold_duration` 秒は速度0で待機(瞬間的なロスト対策)
2. **RETURN**: 検出できていた間の odom 位置履歴から、ロストの `recovery_lookback` 秒前の位置へ P制御の速度指令で戻る
3. **GIVE_UP**: `recovery_timeout` 秒経っても再検出できなければ速度0を保持

再検出したら PID の積分・微分をリセットして通常制御に戻る。
`/target/hole_tracking_valid` が `valid_timeout` 秒途絶えた場合もトラッカ停止とみなしてロスト扱いにする。

### 3.2 hole_world_estimate_compensator.py (p* 推定版)

通常の制御は 3.1 と同じ。それに加えて、穴は壁に固定されているので

```
p* = p + a · d     (p: 機体位置[m], d: 画素偏差[px], a = −scale [m/px])
```

で「穴を基準点に合わせられる機体のworld位置 p*」を毎フレーム計算し、平滑化して保持する。
ロスト時は p* が十分推定できていれば p* へ、そうでなければ 3.1 と同じく履歴の位置へ戻る。

換算係数 `scale` (= 壁までの距離 / 焦点距離) の決め方は2通り:

| モード | 設定 | 内容 |
|---|---|---|
| オンライン推定 (既定) | `~use_online_scale:=true` | 飛行中の odom 変位と画素変化から最小二乗で推定。`initial_scale_*` が初期値 |
| 固定 (推奨) | `~scale_file:=<yaml> ~use_online_scale:=false` | 3.3 のキャリブレーション値を固定で使う |

前提: 壁の法線が world の x 軸にほぼ一致 (yaw≈0) し、壁までの距離が毎回ほぼ同じ。
距離の誤差はそのまま scale の誤差(割合)になるが、ロスト時の目的は「穴を視野内に戻す」ことなので数cm程度の誤差は許容できる。

確認用トピック:

- `/hole_world_estimate/goal` (PointStamped): y, z = p*、x = 平均に使ったサンプル数
- `/hole_world_estimate/scale` (Vector3): x, y = y軸/z軸の scale [m/px]、z = 係数の更新回数

機体を動かしても `goal` の y, z がほぼ一定なら推定は正しい。

### 3.3 hole_pixel_scale_calibration.py (事前キャリブレーション)

トラッキング中にホバリングした位置を原点として、`simple_nav` の POS_MODE で y, z をそれぞれ
`offsets` の各点へ動かし、静定後に odom 位置と画素偏差を平均する。
直線 `d = p / scale + c` を当てはめて scale を求め、YAML に保存する。

- 表示・保存される指標: `scale`, `r2`, `cross_coupling` (もう一方の画素軸への漏れ), `max_residual_px`, `num_points`, `pos_range_m`
- `r2 < 0.95` や `|cross_coupling| > 0.2` で警告(レンズ歪み、yaw ずれの可能性)
- 傾きが負(補償器の符号の前提と逆)の場合や有効点が `min_points` 未満の場合は保存しない
- 各点でトラッキング有効フレームが `min_valid_ratio` 未満ならその点は捨てる

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `~offsets` | [0, .02, .04, .02, 0, −.02, −.04, −.02, 0] | 原点からの移動量 [m] (y, z それぞれで実施) |
| `~max_offset` | 0.06 | offsets の上限 [m] |
| `~settle_pos_tol` / `~settle_vel_tol` | 0.005 / 0.01 | 静定判定 [m], [m/s] |
| `~settle_min_time` / `~settle_timeout` | 1.5 / 6.0 | 静定待ち時間 [s] |
| `~sample_time` | 1.0 | 各点の平均時間 [s] |
| `~output_path` | `camera/config/hole_pixel_scale.yaml` | 保存先 |
| `~wait_for_enter` | true | 開始前に Enter 待ち |

終了後(Ctrl-C で中断した場合も)原点へ戻る。キャリブレーション中は y が位置制御モードになるが、
補償器を起動すると VEL_MODE の送信で速度制御に戻る。

## 4. 使い方

### 4.1 準備

1. 機体・カメラを起動 (例: `roslaunch gimbalrotor bringup.launch ... airframe:=beatle` とカメラドライバ)
2. 離陸して穴の前でホバリング (`rosrun aerial_robot_base keyboard_command.py` で r → t)
3. トラッカを起動し、`s` → 穴をクリック → 必要なら `c` → 目標位置をクリック

```bash
rosrun camera hole_target_tracker.py            # 圧縮画像なら _compressed:=true
```

### 4.2 (初回・配置を変えたとき) キャリブレーション

補償器は起動しない状態で実行する。

```bash
rosrun camera hole_pixel_scale_calibration.py
# 最初は小さい移動量で確認するとよい
rosrun camera hole_pixel_scale_calibration.py _offsets:="[0.0, 0.01, 0.02, 0.01, 0.0, -0.01, -0.02, -0.01, 0.0]"
```

結果の `r2` と `num_points` を確認する。点が多く捨てられている(`num_points` が少ない、`pos_range_m` が offsets より狭い)場合は、
移動中に穴が視野外に出ているので offsets を小さくするか基準点を画面中央寄りにする。

### 4.3 補償制御

どちらか1つを起動する。

```bash
# シンプル版
rosrun camera hole_deviation_compensator.py

# p* 推定版 (キャリブレーション値を固定で使用)
rosrun camera hole_world_estimate_compensator.py \
  _scale_file:=$(rospack find camera)/config/hole_pixel_scale.yaml _use_online_scale:=false
```

### 4.4 モニタリング

```bash
rqt_plot /target/hole_deviation/x /target/hole_deviation/y
rqt_plot /y_pid_term/total[0] /z_pid_term/total[0]
rqt_plot /hole_world_estimate/goal/point/y /hole_world_estimate/goal/point/z
rosrun image_view image_view image:=/processed_image/hole_tracking
```

## 5. 補償器の主なパラメータ

| パラメータ | 既定値 | 対象 | 内容 |
|---|---|---|---|
| `~kp_y` `~ki_y` `~kd_y` | 1e-4, 1e-6, 0 | 両方 | y (画像横) の PID |
| `~kp` `~ki` `~kd` | 1e-4, 1e-6, 0 | 両方 | z (画像縦) の PID |
| `~limit_sum_y` / `~limit_sum` | 0.02 | 両方 | 速度指令上限 [m/s] |
| `~enable_recovery` | true | 両方 | false でロスト時は速度0のみ |
| `~hold_duration` | 0.5 | 両方 | HOLD 時間 [s] |
| `~recovery_lookback` | 0.5 | 両方 | 履歴で遡る時間 [s] |
| `~recovery_timeout` | 10.0 | 両方 | 復帰を諦めるまでの時間 [s] |
| `~recovery_kp` / `~recovery_limit_vel` | 0.5 / 0.03 | 両方 | 帰還時の P ゲイン [1/s] / 速度上限 [m/s] |
| `~recovery_tolerance` | 0.01 | 両方 | 帰還完了判定 [m] |
| `~valid_timeout` | 0.5 | 両方 | valid 途絶でロスト扱いにする時間 [s] |
| `~scale_file` / `~use_online_scale` | "" / true | p*版 | 換算係数の読み込み / オンライン推定の有無 |
| `~initial_scale_y` / `~initial_scale_z` | 6e-4 | p*版 | 換算係数の初期値 [m/px] |
| `~image_delay` | 0.05 | p*版 | 画像遅延 [s] (この分過去の odom と対応付け) |
| `~goal_filter_alpha` / `~goal_min_samples` | 0.1 / 30 | p*版 | p* の平滑化係数 / 信用するまでのサンプル数 |
| `~goal_outlier_thresh` | 0.05 | p*版 | p* の外れ値除去閾値 [m] |
| `~max_pixel_jump` | 80 | p*版 | 1フレームでこれ以上跳んだら基準点変更とみなし推定リセット [px] |

## 6. 注意点

- 後段ノードは同時に1つだけ。キャリブレーション中に補償器を動かさない。
- odom は mocap (EXPERIMENT_ESTIMATE / GROUND_TRUTH) が望ましい。気圧計ベースの z はドリフトし、帰還位置や scale がずれる。
- トラッカで `c` により基準点を変えると偏差が不連続に変わる。p*版は自動でリセットするが、履歴に基づく帰還先は基準点変更前の位置のままなので注意。
- カメラの解像度・ズーム・取り付け位置、壁までの距離を変えたらキャリブレーションし直す。
- 画像軸と機体軸の向き(符号)は「画像の右/下が正 ⇔ 機体の y/z を負方向へ動かすと偏差が減る」前提。カメラの向きを変えた場合はキャリブレーションで傾きの符号を確認する。
