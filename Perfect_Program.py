#!/usr/bin/env pybricks-micropython
"""
LEGO Spike用 高性能ライントレースプログラム
Pybricksを使用したPID制御による滑らかなライン追従
ポート構成: A,Bにモーター、Dにカラーセンサー
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ハブとセンサーの初期化
hub = PrimeHub()

# モーターの設定
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

# カラーセンサーの設定（ポートDに接続）
color_sensor = ColorSensor(Port.D)

# DriveBaseの設定
# wheel_diameter: ホイールの直径（mm）
# axle_track: 左右のホイール間の距離（mm）
wheel_diameter = 56  # 標準的なLEGOホイールの直径（実際の値に調整してください）
axle_track = 114     # 車軸間の距離（実際の値に調整してください）
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

class LineTracer:
    """高性能ライントレースクラス"""
    
    def __init__(self):
        # センサーのキャリブレーション値
        self.black_threshold = 10  # 黒の反射率の閾値
        self.white_threshold = 85  # 白の反射率の閾値
        self.target_value = (self.black_threshold + self.white_threshold) / 2
        
        # PID制御パラメータ
        self.kp = 2.0   # 比例ゲイン（センサー1つなので少し高めに設定）
        self.ki = 0.02  # 積分ゲイン
        self.kd = 0.8   # 微分ゲイン
        
        # PID制御用変数
        self.integral = 0
        self.last_error = 0
        self.integral_limit = 100  # 積分項の上限
        
        # 走行パラメータ
        self.base_speed = 150  # 基本速度 (mm/s)
        self.max_turn_rate = 250  # 最大回転速度 (deg/s)
        
        # エラー履歴（振動検出用）
        self.error_history = []
        self.history_size = 10
        
        # タイマー
        self.timer = StopWatch()
        
        # キャリブレーションフラグ
        self.is_calibrated = False
        
        # ライン位置（左:-1、中央:0、右:1）
        self.line_position = 0
    
    def calibrate(self):
        """センサーのキャリブレーション"""
        print("=== キャリブレーション開始 ===")
        hub.display.char("C")
        
        # 白色のキャリブレーション
        hub.display.char("W")
        hub.speaker.beep(1000, 100)
        print("センサーを白い地面の上に置いてください")
        print("中央ボタンを押してください")
        
        # ボタンが押されるまで現在の値を表示
        while not Button.CENTER in hub.buttons.pressed():
            current_value = color_sensor.reflection()
            print(f"現在の反射率: {current_value}%", end='\r')
            wait(50)
        
        white_value = color_sensor.reflection()
        print(f"\n白の値: {white_value}%")
        hub.speaker.beep(1500, 100)
        wait(500)
        
        # 黒色のキャリブレーション
        hub.display.char("B")
        hub.speaker.beep(1000, 100)
        print("センサーを黒いラインの上に置いてください")
        print("中央ボタンを押してください")
        
        # ボタンが押されるまで現在の値を表示
        while not Button.CENTER in hub.buttons.pressed():
            current_value = color_sensor.reflection()
            print(f"現在の反射率: {current_value}%", end='\r')
            wait(50)
        
        black_value = color_sensor.reflection()
        print(f"\n黒の値: {black_value}%")
        hub.speaker.beep(1500, 100)
        
        # 閾値の計算と設定
        self.black_threshold = black_value + 5
        self.white_threshold = white_value - 5
        self.target_value = (self.black_threshold + self.white_threshold) / 2
        
        # グレーゾーンの幅を計算（エッジ検出用）
        self.gray_zone = (self.white_threshold - self.black_threshold) * 0.3
        
        print(f"目標値: {self.target_value}%")
        print(f"黒閾値: {self.black_threshold}%")
        print(f"白閾値: {self.white_threshold}%")
        print("=== キャリブレーション完了 ===")
        
        hub.display.char("R")
        hub.speaker.beep(2000, 200)
        wait(1000)
        
        self.is_calibrated = True
    
    def calculate_pid(self, sensor_value):
        """PID制御の計算"""
        # エラーの計算（センサー値と目標値の差）
        error = sensor_value - self.target_value
        
        # エラー履歴の更新（振動検出用）
        self.error_history.append(error)
        if len(self.error_history) > self.history_size:
            self.error_history.pop(0)
        
        # 積分項の計算（エラーの累積）
        self.integral += error
        
        # 積分項の制限（ワインドアップ防止）
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        
        # 振動を検出したら積分項をリセット
        if self.detect_oscillation():
            self.integral *= 0.5
        
        # 微分項の計算（エラーの変化率）
        derivative = error - self.last_error
        
        # PID出力の計算
        turn_rate = (self.kp * error + 
                    self.ki * self.integral + 
                    self.kd * derivative)
        
        # 最大回転速度の制限
        if turn_rate > self.max_turn_rate:
            turn_rate = self.max_turn_rate
        elif turn_rate < -self.max_turn_rate:
            turn_rate = -self.max_turn_rate
        
        # 次回のために現在のエラーを保存
        self.last_error = error
        
        return turn_rate
    
    def detect_oscillation(self):
        """振動の検出"""
        if len(self.error_history) < self.history_size:
            return False
        
        # 符号の変化回数をカウント
        sign_changes = 0
        for i in range(1, len(self.error_history)):
            if self.error_history[i] * self.error_history[i-1] < 0:
                sign_changes += 1
        
        # 振動していると判定
        return sign_changes > self.history_size * 0.6
    
    def detect_sharp_curve(self, sensor_value):
        """急カーブの検出"""
        # センサー値が極端に黒または白に近い場合は急カーブ
        if sensor_value < self.black_threshold + 10:
            return "sharp_left"
        elif sensor_value > self.white_threshold - 10:
            return "sharp_right"
        return None
    
    def search_line(self):
        """ラインを見失った時の探索動作"""
        print("ラインを見失いました。探索中...")
        hub.display.char("?")
        
        # 最後に検出した方向を優先して探索
        if self.line_position < 0:
            search_pattern = [30, -60, 90, -120, 150, -180]
        else:
            search_pattern = [-30, 60, -90, 120, -150, 180]
        
        for angle in search_pattern:
            robot.turn(angle)
            
            # センサー値をチェック
            reflection = color_sensor.reflection()
            if reflection < self.target_value:
                print("ラインを発見しました")
                hub.speaker.beep(2000, 100)
                hub.display.char("R")
                # ラインの位置を更新
                self.line_position = -1 if angle > 0 else 1
                return True
            
            wait(50)
        
        # ラインが見つからない場合
        print("ラインが見つかりません")
        robot.stop()
        hub.speaker.beep(500, 500)
        hub.display.char("X")
        return False
    
    def follow_line(self):
        """ライントレース実行"""
        if not self.is_calibrated:
            print("先にキャリブレーションを実行してください")
            self.calibrate()
        
        print("=== ライントレース開始 ===")
        print("停止するには中央ボタンを押してください")
        hub.display.char("G")
        hub.speaker.beep(1500, 100)
        
        self.timer.reset()
        lost_line_count = 0
        
        while True:
            # 停止ボタンのチェック
            if Button.CENTER in hub.buttons.pressed():
                break
            
            # センサー値の取得
            sensor_value = color_sensor.reflection()
            
            # デバッグ情報の表示（必要に応じてコメントアウト）
            # print(f"反射率: {sensor_value}%", end='\r')
            
            # ラインを完全に見失った場合の処理
            if sensor_value > self.white_threshold + 5:
                lost_line_count += 1
                if lost_line_count > 5:  # 連続して見失った場合
                    if not self.search_line():
                        break
                    lost_line_count = 0
            else:
                lost_line_count = 0
            
            # 急カーブの検出と処理
            curve_type = self.detect_sharp_curve(sensor_value)
            if curve_type:
                if curve_type == "sharp_left":
                    # 左に急旋回
                    robot.drive(self.base_speed * 0.5, -self.max_turn_rate * 0.8)
                    self.line_position = -1
                elif curve_type == "sharp_right":
                    # 右に急旋回
                    robot.drive(self.base_speed * 0.5, self.max_turn_rate * 0.8)
                    self.line_position = 1
            else:
                # 通常のPID制御
                turn_rate = self.calculate_pid(sensor_value)
                
                # 速度の調整（カーブで減速）
                speed_factor = 1 - (abs(turn_rate) / self.max_turn_rate) * 0.4
                current_speed = self.base_speed * speed_factor
                
                # ロボットの駆動
                robot.drive(current_speed, turn_rate)
                
                # ライン位置の更新
                if abs(sensor_value - self.target_value) < self.gray_zone:
                    self.line_position = 0  # 中央
                elif sensor_value < self.target_value:
                    self.line_position = -1  # 左
                else:
                    self.line_position = 1  # 右
            
            # 制御周期の待機
            wait(5)  # より高速な制御ループ
        
        # 停止処理
        robot.stop()
        elapsed_time = self.timer.time() / 1000
        print(f"\n=== ライントレース終了 ===")
        print(f"走行時間: {elapsed_time:.2f}秒")
        hub.display.char("S")
        hub.speaker.beep(1000, 200)
    
    def test_sensors(self):
        """センサーのテストモード"""
        print("=== センサーテストモード ===")
        print("中央ボタンで終了")
        hub.display.char("T")
        
        while not Button.CENTER in hub.buttons.pressed():
            reflection = color_sensor.reflection()
            color = color_sensor.color()
            
            # 反射率に基づいて表示を変更
            if reflection < self.black_threshold:
                status = "黒"
                hub.display.char("B")
            elif reflection > self.white_threshold:
                status = "白"
                hub.display.char("W")
            else:
                status = "グレー"
                hub.display.char("G")
            
            print(f"反射率: {reflection}% | 色: {color} | 状態: {status}", end='\r')
            wait(100)
        
        print("\nテスト終了")
    
    def advanced_settings(self):
        """詳細設定の調整"""
        print("=== 詳細設定 ===")
        print("ボタンで値を調整:")
        print("左/右: Kp調整")
        print("上/下: 速度調整")
        print("中央: 設定完了")
        
        hub.display.char("P")
        
        while True:
            buttons = hub.buttons.pressed()
            
            if Button.LEFT in buttons:
                self.kp -= 0.1
                print(f"Kp: {self.kp:.1f}")
                wait(200)
            elif Button.RIGHT in buttons:
                self.kp += 0.1
                print(f"Kp: {self.kp:.1f}")
                wait(200)
            elif Button.UP in buttons:
                self.base_speed += 10
                print(f"速度: {self.base_speed}")
                wait(200)
            elif Button.DOWN in buttons:
                self.base_speed -= 10
                if self.base_speed < 50:
                    self.base_speed = 50
                print(f"速度: {self.base_speed}")
                wait(200)
            elif Button.CENTER in buttons:
                break
            
            wait(10)
        
        hub.speaker.beep(2000, 200)
        print("設定完了")
        print(f"最終設定 - Kp: {self.kp:.1f}, 速度: {self.base_speed}")

def main():
    """メインプログラム"""
    # ライントレーサーの初期化
    tracer = LineTracer()
    
    # 起動音
    hub.speaker.beep(1000, 100)
    wait(100)
    hub.speaker.beep(1500, 100)
    wait(100)
    hub.speaker.beep(2000, 100)
    
    print("=== LEGO Spike ライントレースプログラム ===")
    print("ポート構成:")
    print("  - ポートA: 左モーター")
    print("  - ポートB: 右モーター")
    print("  - ポートD: カラーセンサー")
    print("")
    
    # メニュー選択
    print("モード選択:")
    print("左ボタン: センサーテスト")
    print("右ボタン: 詳細設定")
    print("中央ボタン: ライントレース開始")
    print("上ボタン: キャリブレーションのみ")
    
    hub.display.char("M")
    wait(500)
    
    # ボタン入力待ち
    while True:
        buttons = hub.buttons.pressed()
        
        if Button.LEFT in buttons:
            # センサーテスト
            tracer.test_sensors()
            break
        elif Button.RIGHT in buttons:
            # キャリブレーション後、詳細設定
            tracer.calibrate()
            tracer.advanced_settings()
            tracer.follow_line()
            break
        elif Button.CENTER in buttons:
            # 通常のライントレース
            tracer.calibrate()
            tracer.follow_line()
            break
        elif Button.UP in buttons:
            # キャリブレーションのみ
            tracer.calibrate()
            print("キャリブレーション完了。中央ボタンでライントレース開始")
            while not Button.CENTER in hub.buttons.pressed():
                wait(10)
            tracer.follow_line()
            break
        
        wait(10)
    
    print("プログラム終了")

# プログラムの実行
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        robot.stop()
        hub.speaker.beep(500, 1000)
        # エラー表示
        hub.display.char("E")