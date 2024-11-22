import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

l0 = 150.0  # モーター間の距離 (mm)
l1 = 90.0   # 第一リンクの長さ (mm)
l2 = 140.0  # 第二リンクの長さ (mm)

# 定数
square_size = 100.0  # 正方形や三角形のサイズ (mm)

# キーポイントの定義
x_center = l0 / 2.0  # 中心のx座標
y_center = 125       # 中心のy座標

# 左端と右端のx座標
x_left = x_center - square_size / 2.0
x_right = x_center + square_size / 2.0

# 下端と上端のy座標
y_bottom = y_center - square_size / 2.0
y_top = y_center + square_size / 2.0

# 各点の座標を定義
RB = (x_right, y_bottom)  # 右下
LB = (x_left, y_bottom)   # 左下
RT = (x_right, y_top)     # 右上
LT = (x_left, y_top)      # 左上

# パスの点数
num_points_line = 200      # 各直線の点数
num_points_arc = int((np.pi / 2.0) * num_points_line)  # 円弧の点数

# 1. RB -> RT まで直線
x_line1 = np.full(num_points_line, x_right)
y_line1 = np.linspace(y_bottom, y_top, num_points_line)

# 2. RT -> LB まで、RBを中心とした半径がsquare_sizeの円弧
theta2 = np.linspace(np.pi/2, np.pi, num_points_arc)
x_line2 = x_right + square_size * np.cos(theta2)
y_line2 = y_bottom + square_size * np.sin(theta2)

# 3. LB -> RB まで直線
x_line3 = np.linspace(x_left, x_right, num_points_line)
y_line3 = np.full(num_points_line, y_bottom)

# 4. RB -> LT まで、LBを中心とした半径がsquare_sizeの円弧
theta4 = np.linspace(0, np.pi/2, num_points_arc)
x_line4 = x_left + square_size * np.cos(theta4)
y_line4 = y_bottom + square_size * np.sin(theta4)

# 5. LT -> LB まで直線
x_line5 = np.full(num_points_line, x_left)
y_line5 = np.linspace(y_top, y_bottom, num_points_line)

# 6. LB -> RT まで、LTを中心とした半径がsquare_sizeの円弧
theta6 = np.linspace(-np.pi/2, 0, num_points_arc)
x_line6 = x_left + square_size * np.cos(theta6)
y_line6 = y_top + square_size * np.sin(theta6)

# 7. RT -> LT まで直線
x_line7 = np.linspace(x_right, x_left, num_points_line)
y_line7 = np.full(num_points_line, y_top)

# 8. LT -> RB まで、RTを中心とした半径がsquare_sizeの円弧
theta8 = np.linspace(np.pi, 3*np.pi/2, num_points_arc)
x_line8 = x_right + square_size * np.cos(theta8)
y_line8 = y_top + square_size * np.sin(theta8)

# パスを連結して一筆書きのパスを生成
x_targets = np.concatenate([
	x_line1, x_line2, x_line3, x_line4,
	x_line5, x_line6, x_line7, x_line8
])
y_targets = np.concatenate([
	y_line1, y_line2, y_line3, y_line4,
	y_line5, y_line6, y_line7, y_line8
])

print("length of x_targets: ", len(x_targets))
print("length of y_targets: ", len(y_targets))

# xとyを結合して一つの配列にする
coordinates = np.column_stack((x_targets, y_targets))

# CSVファイルに保存（ヘッダーなし、カンマ区切り）
np.savetxt('arc_square.csv', coordinates, delimiter=',')

# 経路をプロットして確認（オプション）
plt.plot(x_targets, y_targets)
plt.axis('equal')
plt.title('Generated Path')
plt.xlabel('X-coordinate (mm)')
plt.ylabel('Y-coordinate (mm)')
plt.grid(True)
plt.show()
