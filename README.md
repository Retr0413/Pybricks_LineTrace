# LEGO Spike ライントレース 段階的学習プログラム

## 📖 概要
このプロジェクトは、LEGO Spike PrimeとPybricksを使用して、ライントレースプログラムを段階的に学習するための教材です。レベル1の基本プログラムから始めて、機能を追加しながらレベル4の高性能プログラムまで発展させていきます。

## 🎯 学習目標
- プログラミングの基礎から応用まで段階的に習得
- 制御工学（PID制御）の実践的な理解
- 問題解決能力とエンジニアリング思考の育成

## 📁 ファイル構成
```
level1_basic.py         # レベル1: 基本ライントレース
level2_calibration.py   # レベル2: キャリブレーション機能追加
level3_proportional.py  # レベル3: 比例制御（P制御）追加
level4_pid_complete.py  # レベル4: 完全なPID制御と高度な機能
```

## 🔧 ハードウェア構成
- **ポートA**: 左モーター
- **ポートB**: 右モーター
- **ポートD**: カラーセンサー

## 📊 レベル別機能比較表

| 機能 | レベル1 | レベル2 | レベル3 | レベル4 |
|------|---------|---------|---------|---------|
| 基本走行 | ✅ | ✅ | ✅ | ✅ |
| キャリブレーション | ❌ | ✅ | ✅ | ✅ |
| 比例制御（P） | ❌ | ❌ | ✅ | ✅ |
| 積分制御（I） | ❌ | ❌ | ❌ | ✅ |
| 微分制御（D） | ❌ | ❌ | ❌ | ✅ |
| 速度調整 | ❌ | ❌ | ✅ | ✅ |
| ライン探索 | ❌ | ❌ | ❌ | ✅ |
| 急カーブ対応 | ❌ | ❌ | ❌ | ✅ |
| 走行時間測定 | ❌ | ❌ | ✅ | ✅ |

## 🚀 レベル1: 基本ライントレース

### 実装内容
```python
# 固定パラメータを使用
THRESHOLD = 50      # 白黒の境界値（固定値）
BASE_SPEED = 150    # 基本速度
TURN_RATE = 100     # 旋回速度

# シンプルなON/OFF制御
if reflection < THRESHOLD:
    robot.drive(BASE_SPEED, -TURN_RATE)  # 黒→左旋回
else:
    robot.drive(BASE_SPEED, TURN_RATE)   # 白→右旋回
```

### 特徴
- 最もシンプルな実装
- 固定値（マジックナンバー）を使用
- ON/OFF制御でジグザグ走行

### 問題点
- 🔴 環境光が変わると動作しない
- 🔴 カクカクした動き
- 🔴 速度が一定でカーブで不安定


## 🎨 レベル2: キャリブレーション機能追加

### 新規追加機能

#### 1. calibrate()関数
```python
def calibrate():
    # 白と黒の値を測定
    white_value = color_sensor.reflection()  # 白測定
    black_value = color_sensor.reflection()  # 黒測定

    # 自動的にしきい値を計算
    threshold = (white_value + black_value) / 2
    return threshold, black_value, white_value
```

### 改善点
- ✅ **環境適応**: どんな照明条件でも動作
- ✅ **リアルタイム表示**: キャリブレーション中の値を表示
- ✅ **音声フィードバック**: ビープ音で操作をガイド

### コード差分
```python
# レベル1（変更前）
THRESHOLD = 50  # 固定値

# レベル2（変更後）
THRESHOLD, BLACK_VAL, WHITE_VAL = calibrate()  # 動的に計算
```

### まだ残る問題点
- 🔴 まだカクカクした動き
- 🔴 速度が一定


## ⚙️ レベル3: 比例制御（P制御）追加

### 新規追加機能

#### 1. calculate_p_control()関数
```python
def calculate_p_control(sensor_value, target_value, kp):
    error = sensor_value - target_value  # エラー計算
    turn_rate = kp * error               # 比例制御
    return turn_rate, error
```

#### 2. adjust_speed()関数
```python
def adjust_speed(base_speed, error):
    speed = base_speed - abs(error) * 0.8  # カーブで減速
    if speed < 50:
        speed = 50
    return speed
```

### 新規追加パラメータ
- **KP = 2.0**: 比例ゲイン（反応の強さ）
- **timer**: 走行時間測定用

### 改善点
- ✅ **スムーズな動き**: エラーに比例した制御
- ✅ **カーブで自動減速**: 安定性向上
- ✅ **走行時間測定**: パフォーマンス評価

### コード差分
```python
# レベル2（変更前）
if reflection < THRESHOLD:
    robot.drive(BASE_SPEED, -TURN_RATE)  # 固定旋回
else:
    robot.drive(BASE_SPEED, TURN_RATE)

# レベル3（変更後）
turn_rate, error = calculate_p_control(reflection, TARGET_VALUE, KP)
current_speed = adjust_speed(BASE_SPEED, error)
robot.drive(current_speed, turn_rate)  # 比例制御
```

### まだ残る問題点
- 🔴 直線での微振動
- 🔴 累積誤差の未補正
- 🔴 ライン見失い時の対処なし


## 🏆 レベル4: 完全なPID制御と高度な機能

### 新規追加機能

#### 1. 完全なPID制御
```python
def calculate_pid_control(sensor_value, target_value, kp, ki, kd):
    error = sensor_value - target_value

    # I制御: エラーの累積
    integral += error

    # D制御: エラーの変化率
    derivative = error - last_error

    # PID統合
    turn_rate = (kp * error) + (ki * integral) + (kd * derivative)
    return turn_rate, error
```

#### 2. search_line()関数
```python
def search_line(target_value):
    # ラインを見失った時の探索
    for angle in [30, -60, 90, -120, 150]:
        robot.turn(angle)
        if color_sensor.reflection() < target_value:
            return True  # ライン発見
    return False
```

#### 3. detect_sharp_curve()関数
```python
def detect_sharp_curve(sensor_value, black_thresh, white_thresh):
    # 急カーブの検出
    if sensor_value < black_thresh + 5:
        return "sharp_left"
    elif sensor_value > white_thresh - 5:
        return "sharp_right"
    return None
```

### 新規追加パラメータ
- **KI = 0.02**: 積分ゲイン
- **KD = 0.8**: 微分ゲイン
- **integral**: 積分項の累積値
- **last_error**: 前回のエラー値
- **lost_line_count**: ライン見失いカウンター

### 改善点
- ✅ **振動抑制**: D制御により安定走行
- ✅ **累積誤差補正**: I制御により直線性向上
- ✅ **ライン復帰**: 見失っても自動探索
- ✅ **急カーブ対応**: 特殊処理で確実に曲がる
- ✅ **高速制御**: 5msサイクルで精密制御

### コード差分
```python
# レベル3（変更前）
turn_rate, error = calculate_p_control(...)  # P制御のみ
wait(10)  # 10msサイクル

# レベル4（変更後）
# ライン見失いチェック
if reflection > WHITE_THRESHOLD + 5:
    if not search_line(TARGET_VALUE):
        break

# 急カーブ対応
curve_type = detect_sharp_curve(...)
if curve_type == "sharp_left":
    robot.drive(BASE_SPEED * 0.5, -200)
else:
    # 完全なPID制御
    turn_rate, error = calculate_pid_control(...)

wait(5)  # 5msサイクル
```

## 📈 パフォーマンス比較

| 評価項目 | レベル1 | レベル2 | レベル3 | レベル4 |
|----------|---------|---------|---------|----------|
| 直線の安定性 | ⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| カーブ性能 | ⭐ | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 最高速度 | ⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 環境適応性 | ⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 復帰能力 | ❌ | ❌ | ❌ | ⭐⭐⭐⭐⭐ |

## 🎓 学習のポイント

### レベル1→2で学ぶこと
- 関数の作成と利用
- 動的な値の取得
- 環境への適応の重要性

### レベル2→3で学ぶこと
- フィードバック制御の基礎
- エラー（誤差）の概念
- 比例の数学的理解

### レベル3→4で学ぶこと
- 積分・微分の実用的応用
- 状態管理（グローバル変数）
- エラーハンドリング
- 複雑なシステムの統合