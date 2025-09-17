#!/usr/bin/env pybricks-micropython
"""
レベル3: 比例制御（P制御）を追加
レベル2のコード + 比例制御でスムーズな動き
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
    
    # 目標値を計算（レベル3ではthresholdではなくtargetと呼ぶ）
    target = (white_value + black_value) / 2
    print(f"目標値: {target}%")
    
    hub.display.char("R")
    hub.speaker.beep(2000, 200)
    wait(500)
    
    return target, black_value, white_value

# ========== レベル3で追加: 比例制御関数 ==========
def calculate_p_control(sensor_value, target_value, kp):
    """
    比例制御の計算
    エラー（目標値との差）に比例した制御量を返す
    """
    error = sensor_value - target_value
    turn_rate = kp * error
    return turn_rate, error

# ========== レベル3で追加: 速度調整関数 ==========
def adjust_speed(base_speed, error):
    """
    エラーの大きさに応じて速度を調整
    カーブでは自動的に減速する
    """
    speed = base_speed - abs(error) * 0.8
    if speed < 50:
        speed = 50
    return speed
# ========== 比例制御機能ここまで ==========

# パラメータ
BASE_SPEED = 150    # 基本速度
KP = 2.0           # 比例ゲイン（レベル3で追加）

print("=== レベル3: 比例制御ライントレース ===")

# キャリブレーション実行
TARGET_VALUE, BLACK_VAL, WHITE_VAL = calibrate()

# タイマー開始（レベル3で追加：走行時間測定）
timer = StopWatch()

print("ロボットをラインに置いて中央ボタンを押してください")

# スタート待機
while not Button.CENTER in hub.buttons.pressed():
    wait(10)

print("スタート！（中央ボタンで停止）")
print("比例制御でスムーズに走行します")
hub.speaker.beep(1000, 100)

# メインループ
while True:
    # 停止ボタンチェック
    if Button.CENTER in hub.buttons.pressed():
        break
    
    # センサー値を読む
    reflection = color_sensor.reflection()
    
    # ========== レベル3で変更: ON/OFF制御から比例制御へ ==========
    # 比例制御の計算
    turn_rate, error = calculate_p_control(reflection, TARGET_VALUE, KP)
    
    # 速度調整（カーブで減速）
    current_speed = adjust_speed(BASE_SPEED, error)
    
    # ロボットを駆動（比例制御による滑らかな動き）
    robot.drive(current_speed, turn_rate)
    # ========== 比例制御による駆動ここまで ==========
    
    wait(10)

# 停止
robot.stop()
elapsed_time = timer.time() / 1000
print(f"走行時間: {elapsed_time:.1f}秒")
print("停止しました")
hub.speaker.beep(500, 200)