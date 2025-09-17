#!/usr/bin/env pybricks-micropython
"""
レベル1: 基本的なライントレース
最もシンプルなON/OFF制御で動く基本プログラム
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

# 固定パラメータ
THRESHOLD = 50      # 白黒の境界値（固定値）
BASE_SPEED = 150    # 基本速度
TURN_RATE = 100     # 旋回速度

print("=== レベル1: 基本ライントレース ===")
print("ロボットをラインに置いて中央ボタンを押してください")

# スタート待機
while not Button.CENTER in hub.buttons.pressed():
    wait(10)

print("スタート！（中央ボタンで停止）")
hub.speaker.beep(1000, 100)

# メインループ
while True:
    # 停止ボタンチェック
    if Button.CENTER in hub.buttons.pressed():
        break
    
    # センサー値を読む
    reflection = color_sensor.reflection()
    
    # ON/OFF制御
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