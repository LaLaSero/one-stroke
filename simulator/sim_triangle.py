import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# システムパラメータ
l0 = 150.0  # モーター間の距離 (mm)
l1 = 90.0   # 第一リンクの長さ (mm)
l2 = 140.0  # 第二リンクの長さ (mm)

# PID制御パラメータ
Kp = 13.0
Ki = 0.0
Kd = 0.5

# 制御間隔
control_interval = 0.01  # 秒

# ターゲット更新間隔
target_update_interval = 0.1  # 秒

# PID制御変数 (左腕)
integral1_left = 0.0
prevError1_left = 0.0
integral2_left = 0.0
prevError2_left = 0.0

# PID制御変数 (右腕)
integral1_right = 0.0
prevError1_right = 0.0
integral2_right = 0.0
prevError2_right = 0.0

# 先ほど生成したファイルからx_targetsとy_targetsを読み込み
x_targets_full = np.loadtxt('triangle/x_triangle.txt', delimiter=',')
y_targets_full = np.loadtxt('triangle/y_triangle.txt', delimiter=',')

# 角度差の最小値を計算するヘルパー関数
def angle_difference(a, b):
    """
    二つの角度aとbの間の最小の差を計算します。
    結果は-piからpiの範囲になります。
    """
    diff = a - b
    return np.arctan2(np.sin(diff), np.cos(diff))

# 左腕の逆運動学
def inverse_kinematics_left(x_p, y_p, theta1_current, theta2_current):
    D = (x_p**2 + y_p**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1.0:
        return None, None
    # theta2の可能な解
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

# 右腕の逆運動学
def inverse_kinematics_right(x_p, y_p, theta1_current, theta2_current):
    # 右モーターは(l0, 0)に位置
    x = x_p - l0
    y = y_p
    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1.0:
        return None, None
    # theta2の可能な解
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
    if setpoint is None or measured is None:
        # setpointまたはmeasuredがNoneの場合、計算を避ける
        return 0.0, integral, prev_error
    error = setpoint - measured
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    output = Kp * error + Ki * integral + Kd * derivative
    prev_error = error
    return output, integral, prev_error

# ターゲットインデックスと更新時間の初期化
target_index = 0
next_target_update_time = 0.0  # 初回にすぐ更新するために0.0に設定

# 初期位置
x_p0 = x_targets_full[0]
y_p0 = y_targets_full[0]

# 初期角度 (逆運動学の解に基づく)
result_left = inverse_kinematics_left(x_p0, y_p0, 0.0, -1.0)
result_right = inverse_kinematics_right(x_p0, y_p0, 0.0, 0.0)

if result_left is None or result_right is None:
    print("初期位置が一方または両方のアームの到達範囲外です。")
    exit()
else:
    theta1_left, theta2_left = result_left
    theta1_right, theta2_right = result_right
    print(f"初期左腕角度: theta1 = {np.degrees(theta1_left):.2f}°, theta2 = {np.degrees(theta2_left):.2f}°")
    print(f"初期右腕角度: theta1 = {np.degrees(theta1_right):.2f}°, theta2 = {np.degrees(theta2_right):.2f}°")

# 現在の角度を更新
theta1_current_left = theta1_left
theta2_current_left = theta2_left
theta1_current_right = theta1_right
theta2_current_right = theta2_right

# シミュレーションの初期化
t = 0.0  # シミュレーション時間

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
trail_x = []
trail_y = []

# シミュレーションループ
while target_index < len(x_targets_full):
    # 制御ループの実行

    # ターゲット更新のタイミング
    if t >= next_target_update_time:
        # ターゲットを更新
        x_target = x_targets_full[target_index]
        y_target = y_targets_full[target_index]
        target_index += 1
        next_target_update_time += target_update_interval

    # 左腕と右腕の逆運動学を計算
    result_left = inverse_kinematics_left(x_target, y_target, theta1_current_left, theta2_current_left)
    result_right = inverse_kinematics_right(x_target, y_target, theta1_current_right, theta2_current_right)

    # 逆運動学の解が存在しない場合
    if result_left is None or result_right is None:
        print(f"時刻 {t:.2f}s で位置 ({x_target:.2f}, {y_target:.2f}) に対する逆運動学の解が見つかりません。")
        # 前の角度を維持
        theta1_target_left = theta1_current_left
        theta2_target_left = theta2_current_left
        theta1_target_right = theta1_current_right
        theta2_target_right = theta2_current_right
    else:
        theta1_target_left, theta2_target_left = result_left
        theta1_target_right, theta2_target_right = result_right

    # PID制御を適用して制御信号を生成
    # 左腕
    controlSignal1, integral1_left, prevError1_left = pid_control(
        theta1_target_left, theta1_current_left, integral1_left, prevError1_left, control_interval)
    controlSignal2, integral2_left, prevError2_left = pid_control(
        theta2_target_left, theta2_current_left, integral2_left, prevError2_left, control_interval)

    # 右腕
    controlSignal3, integral1_right, prevError1_right = pid_control(
        theta1_target_right, theta1_current_right, integral1_right, prevError1_right, control_interval)
    controlSignal4, integral2_right, prevError2_right = pid_control(
        theta2_target_right, theta2_current_right, integral2_right, prevError2_right, control_interval)

    # 現在の角度を更新
    theta1_current_left += controlSignal1 * control_interval
    theta2_current_left += controlSignal2 * control_interval
    theta1_current_right += controlSignal3 * control_interval
    theta2_current_right += controlSignal4 * control_interval

    # 角度と位置をリストに保存
    theta1_left_list.append(np.degrees(theta1_current_left))
    theta2_left_list.append(np.degrees(theta2_current_left))
    theta1_right_list.append(np.degrees(theta1_current_right))
    theta2_right_list.append(np.degrees(theta2_current_right))

    # 左腕の末端位置を計算
    x1 = l1 * np.cos(theta1_current_left)
    y1 = l1 * np.sin(theta1_current_left)
    x_p_left = x1 + l2 * np.cos(theta1_current_left + theta2_current_left)
    y_p_left = y1 + l2 * np.sin(theta1_current_left + theta2_current_left)

    # 右腕の末端位置を計算
    x2 = l0 + l1 * np.cos(theta1_current_right)
    y2 = l1 * np.sin(theta1_current_right)
    x_p_right = x2 + l2 * np.cos(theta1_current_right + theta2_current_right)
    y_p_right = y2 + l2 * np.sin(theta1_current_right + theta2_current_right)

    # 両腕のペン位置を平均して一貫性を持たせる
    x_p = (x_p_left + x_p_right) / 2
    y_p = (y_p_left + y_p_right) / 2

    # ペン位置をリストに保存
    x_p_list.append(x_p)
    y_p_list.append(y_p)
    x1_list.append(x1)
    y1_list.append(y1)
    x2_list.append(x2)
    y2_list.append(y2)

    # ペンの軌跡を更新
    trail_x.append(x_p)
    trail_y.append(y_p)

    # 時間を更新
    time_list.append(t)
    t += control_interval

# アニメーションの設定
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-50, l0 + l1 + l2 + 50)
ax.set_ylim(-l1 - l2 - 50, l1 + l2 + 200)
ax.set_xlabel('X Position (mm)')
ax.set_ylabel('Y Position (mm)')
ax.set_title('Robot Arm Simulation')
ax.grid()
ax.axis('equal')

# プロット要素の初期化
line_left_arm, = ax.plot([], [], 'b-o', linewidth=2, label='left arm')
line_right_arm, = ax.plot([], [], 'g-o', linewidth=2, label='right arm')
pen_point, = ax.plot([], [], 'ro', markersize=6, label='pen')
pen_trail, = ax.plot([], [], 'r-', linewidth=1, alpha=0.5, label='actual path')  # 軌跡用のライン
ax.legend()

# アニメーション関数
def animate_func(i):
    if i >= len(x_p_list):
        i = len(x_p_list) - 1

    # 現在のペン位置
    x_p = x_p_list[i]
    y_p = y_p_list[i]

    # 左腕の座標
    x_left = [0, x1_list[i], x_p]
    y_left = [0, y1_list[i], y_p]

    # 右腕の座標
    x_right = [l0, x2_list[i], x_p]
    y_right = [0, y2_list[i], y_p]

    # プロットデータの更新
    line_left_arm.set_data(x_left, y_left)
    line_right_arm.set_data(x_right, y_right)
    pen_point.set_data([x_p], [y_p])  # リストとして渡す

    # ペンの軌跡を更新
    pen_trail.set_data(trail_x[:i+1], trail_y[:i+1])

    return line_left_arm, line_right_arm, pen_point, pen_trail

# ステップ幅を設定してデータを間引き
step = 4
theta1_left_list_sparse = theta1_left_list[::step]
theta1_right_list_sparse = theta1_right_list[::step]
# カンマ区切りで保存
# np.savetxt('degx_sparse.csv', theta1_left_list_sparse, delimiter=',', comments='')
# np.savetxt('degy_sparse.csv', theta1_right_list_sparse, delimiter=',', comments='')

# アニメーションの作成
ani = FuncAnimation(fig, animate_func, frames=len(x_p_list), interval=control_interval*1000, blit=True)
plt.show()

# ジョイント角度の時間変化をプロット
plt.figure(figsize=(12, 6))
plt.plot(time_list, theta1_left_list, label='left arm theta1 (deg)')
plt.plot(time_list, theta1_right_list, label='right arm theta1 (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angles (degrees)')
plt.title('Transitions of Joint Angles')
plt.legend()
plt.grid(True)
plt.show()