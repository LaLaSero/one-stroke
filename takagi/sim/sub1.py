import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# System Parameters
l0 = 100.0  # Distance between motors (mm)
l1 = 75.0  # Length of first link (mm)
l2 = 35.0  # Length of second link (mm)

# PID Control Parameters
Kp = 14.0
Ki = 0.0
Kd = 0.2

# 制御間隔
control_interval = 0.01  # 秒

# エンコーダ設定
CPR = 1000        # 一回転あたりのカウント数
GearRatio = 1.0   # ギア比

# PID制御変数の初期化（左アーム）
integral1_left = 0.0
prevError1_left = 0.0
integral2_left = 0.0
prevError2_left = 0.0

# PID制御変数の初期化（右アーム）
integral1_right = 0.0
prevError1_right = 0.0
integral2_right = 0.0
prevError2_right = 0.0

# 正方形と四分円のパス定義
square_size = 5 # 正方形の一辺の半分の長さ (mm)
num_points_per_side = 75  # 各直線セグメントの補間点数
num_points_per_arc = 75   # 各四分円セグメントの補間点数
arc_radius = square_size  # 四分円の半径 (mm)

# パスの中心をモーター間の中間点に設定
square_center_x = l0 / 2   # 100 mm
square_center_x = 50.0
square_center_y = 80.0

# 正方形の四隅の座標を定義（右上、右下、左下、左上）
square_corners = [
    (square_center_x + square_size, square_center_y + square_size),   # 右上
    (square_center_x + square_size, square_center_y - square_size),   # 右下
    (square_center_x - square_size, square_center_y - square_size),   # 左下
    (square_center_x - square_size, square_center_y + square_size)    # 左上
]

# パスポイントのリスト
path_points = []

# 正方形と四分円を交互に追加
for i in range(len(square_corners)):
    start_corner = square_corners[i]
    end_corner = square_corners[(i + 1) % len(square_corners)]
    
    # 直線セグメントの始点
    path_points.append(start_corner)
    
    # 直線セグメントの終点（四分円の開始点）
    path_points.append(end_corner)
    
    # 四分円の開始角度と終了角度を設定
    if i == 0:
        # 右上から右下への四分円（時計回り）
        theta_start = 0
        theta_end = -np.pi / 2
    elif i == 1:
        # 右下から左下への四分円（時計回り）
        theta_start = -np.pi / 2
        theta_end = -np.pi
    elif i == 2:
        # 左下から左上への四分円（時計回り）
        theta_start = -np.pi
        theta_end = -3 * np.pi / 2
    elif i == 3:
        # 左上から右上への四分円（時計回り）
        theta_start = -3 * np.pi / 2
        theta_end = 0

    # 四分円の点を生成
    theta = np.linspace(theta_start, theta_end, num_points_per_arc, endpoint=False)
    x_arc = square_size * np.cos(theta) + end_corner[0]
    y_arc = square_size * np.sin(theta) + end_corner[1]
    
    # 四分円のポイントをパスに追加
    for x, y in zip(x_arc, y_arc):
        path_points.append((x, y))

# NumPy配列に変換
x_targets = []
y_targets = []
for i in range(len(path_points)):
    start = path_points[i]
    end = path_points[(i + 1) % len(path_points)]
    x_side = np.linspace(start[0], end[0], num_points_per_side, endpoint=False)
    y_side = np.linspace(start[1], end[1], num_points_per_side, endpoint=False)
    x_targets.extend(x_side)
    y_targets.extend(y_side)

x_targets = np.array(x_targets)
y_targets = np.array(y_targets)

# 角度差分を計算するヘルパー関数
def angle_difference(a, b):
    """
    2つの角度a, bの最小差分を計算します。
    差分は-piからpiの範囲に正規化されます。
    """
    diff = a - b
    return np.arctan2(np.sin(diff), np.cos(diff))

# 左アームの逆運動学関数（解の選択を改善）
def inverse_kinematics_left(x_p, y_p, theta1_current, theta2_current):
    D = (x_p**2 + y_p**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1.0:
        return None, None
    # theta2の可能な解を計算
    theta2_options = [np.arctan2(np.sqrt(1 - D**2), D),
                      np.arctan2(-np.sqrt(1 - D**2), D)]
    solutions = []
    for theta2 in theta2_options:
        theta1 = np.arctan2(y_p, x_p) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
        solutions.append((theta1, theta2))
    # 現在の角度に最も近い解を選択
    min_diff = float('inf')
    best_solution = None
    for theta1_sol, theta2_sol in solutions:
        diff1 = abs(angle_difference(theta1_sol, theta1_current))
        diff2 = abs(angle_difference(theta2_sol, theta2_current))
        total_diff = diff1 + diff2
        if total_diff < min_diff:
            min_diff = total_diff
            best_solution = (theta1_sol, theta2_sol)
    return best_solution

# 右アームの逆運動学関数（解の選択を改善）
def inverse_kinematics_right(x_p, y_p, theta1_current, theta2_current):
    # 右モーターは座標(l0, 0)に位置
    x = x_p - l0
    y = y_p
    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1.0:
        return None, None
    # theta2の可能な解を計算
    theta2_options = [np.arctan2(np.sqrt(1 - D**2), D),
                      np.arctan2(-np.sqrt(1 - D**2), D)]
    solutions = []
    for theta2 in theta2_options:
        theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
        solutions.append((theta1, theta2))
    # 現在の角度に最も近い解を選択
    min_diff = float('inf')
    best_solution = None
    for theta1_sol, theta2_sol in solutions:
        diff1 = abs(angle_difference(theta1_sol, theta1_current))
        diff2 = abs(angle_difference(theta2_sol, theta2_current))
        total_diff = diff1 + diff2
        if total_diff < min_diff:
            min_diff = total_diff
            best_solution = (theta1_sol, theta2_sol)
    return best_solution

# PID制御の実装
def pid_control(setpoint, measured, integral, prev_error, dt):
    error = setpoint - measured
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    output = Kp * error + Ki * integral + Kd * derivative
    prev_error = error
    return output, integral, prev_error

# シミュレーションの初期化
time_list = []
theta1_left_list = []
theta2_left_list = []
theta1_right_list = []
theta2_right_list = []
x_p_list = []
y_p_list = []
x1_list = []
y1_list = []
x2_list = []
y2_list = []

# 初期位置の設定
x_p0 = x_targets[0]
y_p0 = y_targets[0]

# 初期角度（現在の角度からスタート）
theta1_current_left = 0.0
theta2_current_left = 0.0
theta1_current_right = 0.0
theta2_current_right = 0.0

# 初期の関節角度を計算
result_left = inverse_kinematics_left(x_p0, y_p0, theta1_current_left, theta2_current_left)
result_right = inverse_kinematics_right(x_p0, y_p0, theta1_current_right, theta2_current_right)

if result_left is None or result_right is None:
    print("Initial position is out of reach for one or both arms.")
    exit()
else:
    theta1_left, theta2_left = result_left
    theta1_right, theta2_right = result_right
    print(f"Initial Left Arm Angles: theta1 = {np.degrees(theta1_left):.2f}°, theta2 = {np.degrees(theta2_left):.2f}°")
    print(f"Initial Right Arm Angles: theta1 = {np.degrees(theta1_right):.2f}°, theta2 = {np.degrees(theta2_right):.2f}°")

# エンコーダカウントの初期化（現在の角度からスタート）
leftEncoderCount = (theta1_left / (2 * np.pi)) * CPR * GearRatio
rightEncoderCount = (theta1_right / (2 * np.pi)) * CPR * GearRatio

# 現在の角度を更新
theta1_current_left = theta1_left
theta2_current_left = theta2_left
theta1_current_right = theta1_right
theta2_current_right = theta2_right

t = 0.0

# 先端部分の軌跡を保存するリスト
trail_x = []
trail_y = []

for x_target, y_target in zip(x_targets, y_targets):
    # 両アームの逆運動学を計算
    result_left = inverse_kinematics_left(x_target, y_target, theta1_current_left, theta2_current_left)
    result_right = inverse_kinematics_right(x_target, y_target, theta1_current_right, theta2_current_right)

    if result_left is None or result_right is None:
        print(f"At time {t:.2f}s, inverse kinematics solution not found for position ({x_target:.2f}, {y_target:.2f})")
        # 前の角度を維持
        theta1_target_left = theta1_current_left
        theta2_target_left = theta2_current_left
        theta1_target_right = theta1_current_right
        theta2_target_right = theta2_current_right
    else:
        theta1_target_left, theta2_target_left = result_left
        theta1_target_right, theta2_target_right = result_right
        print(f"Target Position: ({x_target:.2f}, {y_target:.2f})")
        print(f"Left Arm Target Angles: theta1 = {np.degrees(theta1_target_left):.2f}°, theta2 = {np.degrees(theta2_target_left):.2f}°")
        print(f"Right Arm Target Angles: theta1 = {np.degrees(theta1_target_right):.2f}°, theta2 = {np.degrees(theta2_target_right):.2f}°")

    # 左アームのPID制御
    controlSignal1, integral1_left, prevError1_left = pid_control(
        theta1_target_left, theta1_current_left, integral1_left, prevError1_left, control_interval)
    controlSignal2, integral2_left, prevError2_left = pid_control(
        theta2_target_left, theta2_current_left, integral2_left, prevError2_left, control_interval)

    # 右アームのPID制御
    controlSignal3, integral1_right, prevError1_right = pid_control(
        theta1_target_right, theta1_current_right, integral1_right, prevError1_right, control_interval)
    controlSignal4, integral2_right, prevError2_right = pid_control(
        theta2_target_right, theta2_current_right, integral2_right, prevError2_right, control_interval)

    # 現在の角度を更新
    theta1_current_left += controlSignal1 * control_interval
    theta2_current_left += controlSignal2 * control_interval
    theta1_current_right += controlSignal3 * control_interval
    theta2_current_right += controlSignal4 * control_interval

    # 角度と位置を保存
    theta1_left_list.append(np.degrees(theta1_current_left))
    theta2_left_list.append(np.degrees(theta2_current_left))
    theta1_right_list.append(np.degrees(theta1_current_right))
    theta2_right_list.append(np.degrees(theta2_current_right))

    # アームの末端位置を計算
    # 左アーム
    x1 = l1 * np.cos(theta1_current_left)
    y1 = l1 * np.sin(theta1_current_left)
    x_p_left = x1 + l2 * np.cos(theta1_current_left + theta2_current_left)
    y_p_left = y1 + l2 * np.sin(theta1_current_left + theta2_current_left)

    # 右アーム
    x2 = l0 + l1 * np.cos(theta1_current_right)
    y2 = l1 * np.sin(theta1_current_right)
    x_p_right = x2 + l2 * np.cos(theta1_current_right + theta2_current_right)
    y_p_right = y2 + l2 * np.sin(theta1_current_right + theta2_current_right)

    # 両アームのペン位置を平均して一貫性を保つ
    x_p = (x_p_left + x_p_right) / 2
    y_p = (y_p_left + y_p_right) / 2

    # 位置を保存
    x_p_list.append(x_p)
    y_p_list.append(y_p)
    x1_list.append(x1)
    y1_list.append(y1)
    x2_list.append(x2)
    y2_list.append(y2)

    # 軌跡に現在のペン位置を追加
    trail_x.append(x_p)
    trail_y.append(y_p)

    # 時間を保存
    time_list.append(t)
    t += control_interval

# アニメーションの設定
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-50, l0 + l1 + l2 + 50)
ax.set_ylim(-l1 - l2 - 50, l1 + l2 + 50)
ax.set_xlabel('X Position (mm)')
ax.set_ylabel('Y Position (mm)')
ax.set_title('Robot Arm Movement Animation with Pen Trajectory')
ax.grid()
ax.axis('equal')

# プロット要素の初期化
line_left_arm, = ax.plot([], [], 'b-o', linewidth=2, label='Left Arm')
line_right_arm, = ax.plot([], [], 'g-o', linewidth=2, label='Right Arm')
pen_point, = ax.plot([], [], 'ro', markersize=6, label='Pen')
pen_trail, = ax.plot([], [], 'r-', linewidth=1, alpha=0.5, label='Pen Trajectory')  # 軌跡用ライン
ax.legend()

# アニメーション関数の定義
def animate_func(i):
    if i >= len(x_p_list):
        i = len(x_p_list) - 1

    # 現在のペン位置
    x_p = x_p_list[i]
    y_p = y_p_list[i]

    # 左アームの座標
    x_left = [0, x1_list[i], x_p]
    y_left = [0, y1_list[i], y_p]

    # 右アームの座標
    x_right = [l0, x2_list[i], x_p]
    y_right = [0, y2_list[i], y_p]

    # プロットデータの更新
    line_left_arm.set_data(x_left, y_left)
    line_right_arm.set_data(x_right, y_right)
    pen_point.set_data([x_p], [y_p])  # リストとして渡す

    # ペンの軌跡の更新
    pen_trail.set_data(trail_x[:i+1], trail_y[:i+1])

    return line_left_arm, line_right_arm, pen_point, pen_trail

# アニメーションの作成
ani = FuncAnimation(fig, animate_func, frames=len(x_p_list), interval=control_interval*1000, blit=True)

plt.show()

# 関節角度のグラフをプロット
plt.figure(figsize=(12, 6))
plt.plot(time_list, theta1_left_list, label='Theta1 Left Arm (Degrees)')
plt.plot(time_list, theta2_left_list, label='Theta2 Left Arm (Degrees)')
plt.plot(time_list, theta1_right_list, label='Theta1 Right Arm (Degrees)')
plt.plot(time_list, theta2_right_list, label='Theta2 Right Arm (Degrees)')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angles (Degrees)')
plt.title('Joint Angles Over Time')
plt.legend()
plt.grid(True)
plt.show()
