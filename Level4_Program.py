#!/usr/bin/env pybricks-micropython
"""
レベル4: 完全なPID制御とライン探索機能
レベル3のコード + 積分・微分制御 + ライン探索 + 急カーブ対応
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ハードウェアの初期化
hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
color_sensor = ColorSensor(Port.D)

# DriveBaseの設定
wheel_diameter = 56  # ホイールの直径（mm）
axle_track = 114     # 車軸間の距離（mm）
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# ========== レベル4で追加: PID制御用のグローバル変数 ==========
integral = 0        # 積分項の累積値
last_error = 0      # 前回のエラー値（微分計算用）
integral_limit = 100  # 積分項の制限値
# ========================================================

# キャリブレーション機能（レベル2から継承）
def calibrate():
    """白と黒の値を測定して、自動的にしきい値を設定する"""
    print("=== キャリブレーション開始 ===")
    
    # 白の測定
    hub.display.char("W")
    print("白い地面にセンサーを置いて中央ボタンを押してください")
    while not Button.CENTER in hub.buttons.pressed():
        current = color_sensor.reflection()
        print(f"現在の値: {current}%", end='\r')
        wait(50)
    
    white_value = color_sensor.reflection()
    print(f"\n白の値: {white_value}%")
    hub.speaker.beep(1000, 100)
    wait(500)
    
    # 黒の測定
    hub.display.char("B")
    print("黒いラインにセンサーを置いて中央ボタンを押してください")
    while not Button.CENTER in hub.buttons.pressed():
        current = color_sensor.reflection()
        print(f"現在の値: {current}%", end='\r')
        wait(50)
    
    black_value = color_sensor.reflection()
    print(f"\n黒の値: {black_value}%")
    hub.speaker.beep(1500, 100)
    
    # 目標値と閾値を計算
    target = (white_value + black_value) / 2
    
    # レベル4で追加：閾値も保存（ライン検出用）
    black_threshold = black_value + 5
    white_threshold = white_value - 5
    
    print(f"目標値: {target}%")
    print(f"黒閾値: {black_threshold}%, 白閾値: {white_threshold}%")
    
    hub.display.char("R")
    hub.speaker.beep(2000, 200)
    wait(500)
    
    return target, black_threshold, white_threshold

# ========== レベル4で変更: P制御からPID制御へ ==========
def calculate_pid_control(sensor_value, target_value, kp, ki, kd):
    """
    完全なPID制御の計算
    P: 比例項（現在のエラー）
    I: 積分項（エラーの累積）
    D: 微分項（エラーの変化率）
    """
    global integral, last_error
    
    # エラーの計算
    error = sensor_value - target_value
    
    # 積分項の計算（エラーの累積）
    integral += error
    if integral > integral_limit:
        integral = integral_limit
    elif integral < -integral_limit:
        integral = -integral_limit
    
    # 微分項の計算（エラーの変化率）
    derivative = error - last_error
    
    # PID制御の出力
    turn_rate = (kp * error) + (ki * integral) + (kd * derivative)
    
    # 最大値の制限
    max_turn = 250
    if turn_rate > max_turn:
        turn_rate = max_turn
    elif turn_rate < -max_turn:
        turn_rate = -max_turn
    
    # 次回のために記録
    last_error = error
    
    return turn_rate, error

# ========== レベル4で追加: ライン探索機能 ==========
def search_line(target_value):
    """
    ラインを見失った時の探索動作
    左右に振ってラインを探す
    """
    print("ライン探索中...")
    hub.display.char("?")
    
    search_angles = [30, -60, 90, -120, 150]
    
    for angle in search_angles:
        robot.turn(angle)
        
        # ラインを発見したか確認
        if color_sensor.reflection() < target_value:
            print("ライン発見！")
            hub.speaker.beep(2000, 100)
            hub.display.char("R")
            return True
        
        wait(50)
    
    print("ラインが見つかりません")
    hub.display.char("X")
    return False

# ========== レベル4で追加: 急カーブ検出機能 ==========
def detect_sharp_curve(sensor_value, black_thresh, white_thresh):
    """
    急カーブの検出
    センサー値が極端な場合は急カーブと判定
    """
    if sensor_value < black_thresh + 5:
        return "sharp_left"
    elif sensor_value > white_thresh - 5:
        return "sharp_right"
    return None

# 速度調整関数（レベル3から継承、レベル4で改良）
def adjust_speed(base_speed, error):
    """
    エラーの大きさに応じて速度を調整
    カーブでは自動的に減速する
    """
    speed = base_speed - abs(error) * 0.8
    if speed < 50:
        speed = 50
    return speed

# パラメータ
BASE_SPEED = 150    # 基本速度
KP = 2.0           # 比例ゲイン
KI = 0.02          # 積分ゲイン（レベル4で追加）
KD = 0.8           # 微分ゲイン（レベル4で追加）

print("=== レベル4: 完全なPID制御ライントレース ===")
print("新機能: PID制御、ライン探索、急カーブ対応")

# キャリブレーション実行
TARGET_VALUE, BLACK_THRESHOLD, WHITE_THRESHOLD = calibrate()

# タイマー開始
timer = StopWatch()
lost_line_count = 0  # ライン見失いカウンター（レベル4で追加）

print("スタート！（中央ボタンで停止）")
print("PID制御で最高のパフォーマンスを発揮します")
hub.speaker.beep(1000, 100)

# メインループ
while True:
    # 停止ボタンチェック
    if Button.CENTER in hub.buttons.pressed():
        break
    
    # センサー値を読む
    reflection = color_sensor.reflection()
    
    # ========== レベル4で追加: ライン見失いチェック ==========
    if reflection > WHITE_THRESHOLD + 5:
        lost_line_count += 1
        if lost_line_count > 5:  # 連続で見失った場合
            if not search_line(TARGET_VALUE):
                break  # ライン発見失敗で終了
            lost_line_count = 0
            integral = 0  # 積分項リセット
    else:
        lost_line_count = 0
    
    # ========== レベル4で追加: 急カーブ対応 ==========
    curve_type = detect_sharp_curve(reflection, BLACK_THRESHOLD, WHITE_THRESHOLD)
    
    if curve_type == "sharp_left":
        # 急左カーブ
        robot.drive(BASE_SPEED * 0.5, -200)
        integral = 0  # 積分項リセット
    elif curve_type == "sharp_right":
        # 急右カーブ
        robot.drive(BASE_SPEED * 0.5, 200)
        integral = 0  # 積分項リセット
    else:
        # ========== 通常のPID制御 ==========
        # PID制御の計算（レベル4で完全版）
        turn_rate, error = calculate_pid_control(reflection, TARGET_VALUE, KP, KI, KD)
        
        # 速度調整
        current_speed = adjust_speed(BASE_SPEED, error)
        
        # ロボットを駆動
        robot.drive(current_speed, turn_rate)
    
    wait(5)  # レベル4で高速化: 10ms → 5ms

# 停止
robot.stop()
elapsed_time = timer.time() / 1000
print(f"走行時間: {elapsed_time:.1f}秒")
print("PID制御による高性能ライントレース完了！")
hub.speaker.beep(500, 200)

# ========== レベル4で追加: 統計情報表示 ==========
print(f"\n=== 統計情報 ===")
print(f"最終積分値: {integral:.2f}")
print(f"最終エラー: {last_error:.2f}")
hub.display.char("E")  # End
