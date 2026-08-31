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

---

## 共通する設計パターン
多くのスクリプトが `mask.py` の `/processed_image/filtered_mask`（赤/青などのHLSマスク＋キャリブレーション画像による無効領域除去）をパイプラインの入力とし、そこから円検出・輪郭検出・重心計算・テンプレート追跡などの後段処理に分岐する構成になっている。オプティカルフロー系（`optical_flow*.py`, `feature_track.py`）は独立した系統で、機体自身の動き推定や特定点追跡に使われていると見られる。
