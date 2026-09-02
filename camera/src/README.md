# camera/src スクリプト一覧

`camera/src/` 以下の画像処理スクリプトを機能別にまとめたもの。ROSトピックの購読/配信元も併記する。

## 1. 生画像の確認
- **check_image.py**: `/usb_cam/image_raw/compressed` を購読し、デコードしてウィンドウ表示するだけの最小構成スクリプト。動作確認用。

## 2. 色検出・マスク生成（赤/青）
- **camera_red_detection.py**: `/usb_cam/image_raw/compressed` を購読。HLS変換して赤色2レンジのマスクを生成し、ノズル付近の指定領域を切り出して配信（`/processed_image/cut`, `/mask`, `/lines`）。HoughLinesPやLSD（`pylsd`）による直線検出機能も持つが呼び出しはコメントアウト。
- **color_judge.py**: 同様に `/usb_cam/image_raw/compressed` を購読し、赤・青2色それぞれのHLSマスクを生成（青はさらにモルフォロジー処理でクリーニング）。`/processed_image/red_mask`, `/blue_mask`, `/blue_mask_cleaned` を配信。
- **mask.py**: 汎用の `MaskGenerator` クラス。指定トピック（デフォルト `/usb_cam/image_raw/compressed`）から画像を取得し、HLS単一レンジでマスク生成→モルフォロジークリーニング→キャリブレーション画像（`calib_blackpoint.png`）による無効領域の除外まで行い、`/processed_image/mask`, `/mask_cleaned`, `/filtered_mask` を配信。他スクリプト（edge_detection, get_points, circle_detection等）はこの `filtered_mask` を入力として使う設計。
- **generate_mask_from_file.py**: ROSを使わないオフラインツール。ファイル名とマスクパラメータを標準入力で受け取り、`mask.py` の `MaskGenerator` で単一画像に対してマスク処理をかける（パラメータ調整用）。

## 3. 輪郭・直線検出
- **edge_detection.py**: `/processed_image/filtered_mask` を購読し、二値化＋`findContours` で輪郭抽出、描画結果を `/processed_image/edge` に配信。
- **line_detection.py**: `/processed_image/mask` を購読し、白色点群を極座標変換（`cartesian_to_polar`）して角度ヒストグラムの最頻値から直線の傾きを推定（`choose_line`）。可視化や配信は無し（計算のみ、実質未完成/デバッグ用）。
- **test_line.py**: `line_detection.py` のクラス版・拡張版。`LineDetection` クラスとして同様の極座標変換＋直線推定に加え、半径方向の分布計算、推定直線の描画（`cv2.imshow`）、matplotlibでリアルタイムグラフ表示（角度分布・半径分布のアニメーション）まで行う、デバッグ・可視化に特化したスクリプト。
- **line_detection_random.py**: ROSを使わない単体テストスクリプト。ランダムな点群を生成し、`cartesian_to_polar`→`choose_line` のロジックをmatplotlibで検証する（`line_detection.py` のアルゴリズム確認用）。
- **red_detection.py**: ROSを使わないオフラインスクリプト。画像ファイルを読み込み、赤色マスク生成→Canny＋HoughLinesPで直線検出し、結果を画像ファイルとして保存（`camera_red_detection.py` のアルゴリズムの単体テスト版）。

## 4. 円検出
- **circle_detection.py**: 任意トピック（デフォルト `/processed_image/filtered_mask`）を購読。グレースケール化＋メディアンブラー後、キャリブレーション画像でのマスキングを経て `HoughCircles` で円検出。最大円の中心・半径を `/target/2D_position`（Vector3）として配信、検出結果画像を `/processed_image/circle` に配信。

## 5. 重心・特徴点抽出
- **get_points.py**: `/processed_image/filtered_mask` を購読し、白色ピクセル群の重心を計算（点数が1000未満なら「不十分」と判定）。重心座標を `/target/2D_position` に配信、重心を描画した画像を `/processed_image/center` に配信。

## 6. オプティカルフロー
- **optical_flow.py**: `/usb_cam/image_raw` を購読し、Farneback法で密なオプティカルフローを計算。閾値以上の動きベクトルの平均（大きさ・角度）をコンソール出力し、HSVで可視化（色相=方向、明度=大きさ）。クリックした点をトラッキングする機能もあり。画面中央から平均動きベクトルを矢印表示する機能あり（2026-08-24追加）。
- **optical_flow_feature.py**: `/usb_cam/image_raw` を購読し、Lucas-Kanade法（疎なオプティカルフロー）でクリック指定点を追跡。軌跡を線・点で描画し `cv2.imshow` 表示（配信なし）。
- **optical_flow_point.py**: `optical_flow_feature.py` の発展版。クリックで追跡点を追加する際にキャリブレーション画像で無効領域を除外し、追跡点の座標を `/target/2D_position` に配信。追跡消失時のリセット処理（`r`キー）も持つ。
- **feature_track.py**: ROSを一切使わない、PC内蔵/USBカメラ（`cv2.VideoCapture(0)`）から直接映像を取得してLucas-Kanade法のオプティカルフローを試すスタンドアロンのサンプルコード。

## 7. テンプレートマッチング/追跡
- **tracker.py**: `/usb_cam/image_raw`（既定）を購読。マウスクリックまたは `selectROI`、あるいは既存のROIファイル読み込みでテンプレート（ROI画像）を設定し、`matchTemplate` によるテンプレートマッチングでフレーム内の対象を追跡。スコアが閾値超なら結果を `/target/2D_position` に配信し、次フレームのROIも自動更新（トラッキング継続）。

## 8. 壁全体との位置合わせ
- **wall_alignment_tracker.py**: `tracker.py`が1点のROIをテンプレートとして追跡するのに対し、こちらは壁全体の見た目を基準画像として使い、現在のカメラ画像が基準（望ましい状態）からどれだけズレているかを推定する。`/usb_cam/image_raw`（既定）を購読し、基準画像に対して`cv2.findTransformECC`（`MOTION_EUCLIDEAN`、並進＋回転）で位置合わせを行う。壁の模様が固有（繰り返しでない）かつズレが小さいことを前提に、前フレームの推定結果を初期値としたウォームスタートでリアルタイム性を確保している。
  - 基準画像は起動時にファイル（既定 `src/image/wall_reference.png`、`~ref_path`パラメータで変更可）から読み込む。ファイルが無い場合や再基準化したい場合は、表示ウィンドウ上で `s` キーを押すと現在のフレームを新しい基準として保存・反映する。
  - ズレ量（横・縦・回転）を `/wall_alignment/deviation`（`geometry_msgs/Pose2D`）に配信。ECCの相関係数が閾値未満（`~min_correlation`、既定0.6）の場合は位置合わせ失敗とみなし `/wall_alignment/valid`（`std_msgs/Bool`）に`false`を配信する。
  - デバッグ用画像を `/processed_image/wall_alignment` に配信。画面中心からのズレ方向・大きさを矢印で表示し、dx/dy/dtheta・相関係数の数値もオーバーレイ表示する。
  - 主なパラメータ: `~compressed`（`true`にすると`sensor_msgs/CompressedImage`型で購読、既定`false`＝`sensor_msgs/Image`型）, `~topic`（購読トピック、未指定時は`~compressed`の値に応じて`/usb_cam/image_raw`または`/usb_cam/image_raw/compressed`を自動選択）, `~ref_path`（基準画像パス）, `~resize_width`/`~resize_height`（ECC計算時の縮小解像度、既定320x240）, `~max_iterations`/`~epsilon`（ECCの収束条件）, `~min_correlation`（有効と判定する最小相関係数）。
  - 実行例: `rosrun camera wall_alignment_tracker.py _compressed:=true`（`/usb_cam/image_raw/compressed`を購読）
  - 将来、壁との距離（スケール変化）まで検出したくなった場合は`MOTION_EUCLIDEAN`を`MOTION_AFFINE`に切り替えることで対応可能。

## 9. 黒いエリア（対象ボード）の下の縁検出
- **board_edge_detection.py**: 壁面の黒いエリア（中心に円形の穴があるボード）の外周のうち、下側の縁だけを購読中の生画像から抽出する。`/usb_cam/image_raw`（既定、`~compressed:=true`で`/usb_cam/image_raw/compressed`）を購読する。
  - **探索範囲の限定**: 下の縁は画像の下側にしか現れない前提で、`~search_top_ratio`（既定0.5＝下半分）より下の行だけを処理対象にする。これにより上半分に映り込むロボットや背景を誤って拾う可能性を減らし、計算量も削減する。
  - **二値化としきい値**: グレースケール化＋メディアンブラー後、黒いエリアだけが暗いグレースケール値になる前提で二値化する。既定では`~auto_threshold`（既定true）によりOtsuの二値化でフレームごとに黒/非黒の境界となるしきい値を自動計算し、`~min_black_thresh`〜`~max_black_thresh`（既定20〜100）の範囲にクリップして使う。Otsuは現在のフレーム自身の明暗ヒストグラムから谷を探すため、現場が暗い（全体の明るさが下がる）場合でもしきい値がその都度追従し、`~black_thresh`を固定値で運用するより頑健。`~auto_threshold:=false`にすると固定しきい値`~black_thresh`（既定80）を使う従来動作に戻せる。実際に使われたしきい値はデバッグ画像左上に`black_thresh=...`として表示される。
    - **調整の仕方**: `/processed_image/board_edge`と`/processed_image/board_mask`をrqt_image_view等で見ながら、(1) 黒いエリアが欠けて検出できない→`~max_black_thresh`を上げる、(2) 背景やロボットの映り込みまで黒く誤検出される→`~max_black_thresh`を下げる、(3) 暗すぎる現場でボード自体もほぼ潰れて全く二値化できない→`~min_black_thresh`を下げるか、カメラ側の露光・ゲインを上げることを検討する。特定の照明環境だけで運用するなら`~auto_threshold:=false`にして`~black_thresh`を直接チューニングしてもよい。
  - モルフォロジー処理（オープン→クローズ）でノイズを除去後、`findContours`（`RETR_EXTERNAL`）で外周のみを取り出す（中心の円形の穴は中間〜明るいグレーのため無視される）。面積が`~min_area`（既定2000）未満の輪郭は誤検出とみなして無効判定。
  - 検出した輪郭は`~approx_epsilon_ratio`（既定0.01）で`approxPolyDP`により多角形近似し、各辺について「水平からのズレが`~max_angle_from_horizontal_deg`（既定15度）以内」かつ「長さが`~expected_length`±`~length_tolerance`（既定300px±150px）の範囲内」の2条件を満たす辺だけを候補とし、その中で画像内で最もy座標が大きい（＝最も下側にある）辺を下の縁として選ぶ。ロボットの映り込みで上辺に切れ込みができて水平な短辺が余分に生じても、長さ条件で候補から外れるため下の縁の検出には影響しない。
  - 検出した下の縁の両端点を`/target/board_bottom_edge`（`geometry_msgs/PolygonStamped`、2点）に配信。二値マスクを`/processed_image/board_mask`（mono8、探索範囲外は黒で埋める）、輪郭全体と下の縁を強調描画したデバッグ画像（探索範囲の境界線も表示）を`/processed_image/board_edge`（bgr8）に配信。有効/無効は`/target/board_bottom_edge_valid`（`std_msgs/Bool`）で通知する。
  - 円形の穴の検出（`circle_detection.py`等）は別スクリプトでの後段処理を想定しており、本スクリプトは黒いエリアの下の縁取得のみを担当する。

---

## 共通する設計パターン
多くのスクリプトが `mask.py` の `/processed_image/filtered_mask`（赤/青などのHLSマスク＋キャリブレーション画像による無効領域除去）をパイプラインの入力とし、そこから円検出・輪郭検出・重心計算・テンプレート追跡などの後段処理に分岐する構成になっている。オプティカルフロー系（`optical_flow*.py`, `feature_track.py`）は独立した系統で、機体自身の動き推定や特定点追跡に使われていると見られる。
