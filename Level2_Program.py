#!/usr/bin/env pybricks-micropython
"""
レベル2: キャリブレーション機能を追加
レベル1のコード + 自動キャリブレーション機能
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ハードウェアの初期化
hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
color_sensor = ColorSensor(Port.D)

# DriveBaseの設定
wheel_diameter = 56  # ホイールの直径（mm）
axle_track = 114     # 車軸間の距離（mm）
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# ========== レベル2で追加: キャリブレーション機能 ==========
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
    
    # しきい値を計算
    threshold = (white_value + black_value) / 2
    print(f"計算されたしきい値: {threshold}%")
    
    hub.display.char("R")
    hub.speaker.beep(2000, 200)
    wait(500)
    
    return threshold, black_value, white_value
# ========== キャリブレーション機能ここまで ==========

# パラメータ（キャリブレーションで更新される）
BASE_SPEED = 150    # 基本速度
TURN_RATE = 100     # 旋回速度

print("=== レベル2: キャリブレーション付きライントレース ===")

# キャリブレーション実行（レベル2の新機能）
THRESHOLD, BLACK_VAL, WHITE_VAL = calibrate()

print("ロボットをラインに置いて中央ボタンを押してください")

# スタート待機
while not Button.CENTER in hub.buttons.pressed():
    wait(10)

print("スタート！（中央ボタンで停止）")
hub.speaker.beep(1000, 100)

# メインループ（レベル1と同じ）
while True:
    # 停止ボタンチェック
    if Button.CENTER in hub.buttons.pressed():
        break
    
    # センサー値を読む
    reflection = color_sensor.reflection()
    
    # ON/OFF制御（キャリブレーションされた値を使用）
    if reflection < THRESHOLD:
        # 黒を検出 → 左に曲がる
        robot.drive(BASE_SPEED, -TURN_RATE)
    else:
        # 白を検出 → 右に曲がる
        robot.drive(BASE_SPEED, TURN_RATE)
    
    wait(10)

# 停止
robot.stop()
print("停止しました")
hub.speaker.beep(500, 200)