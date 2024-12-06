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

# 三角形の頂点の高さを追加（正三角形）
triangle_height = square_size * np.sqrt(3) / 2.0  # 正三角形の高さ

# 各点の座標を定義
RB = (x_right, y_bottom)                         # 右下
LB = (x_left, y_bottom)                          # 左下
Vertex = (x_center, y_bottom + triangle_height)  # 正三角形の頂点
RT = (x_right, y_top)                            # 右上
LT = (x_left, y_top)                             # 左上

RLM = (x_center, y_bottom)  # 右下から左下への中間点（下辺の中央）

# パスの点数
num_points_line = 250      # 各直線の点数
num_points_circle = num_points_line * 3  # 円の点数（フルサークルなので直線の点数の4倍）

# 1. Vertex -> LB まで直線
x_line1 = np.linspace(Vertex[0], LB[0], num_points_line)
y_line1 = np.linspace(Vertex[1], LB[1], num_points_line)

# 2. LB -> RB まで直線
x_line2 = np.linspace(LB[0], RB[0], num_points_line*2)
y_line2 = np.linspace(LB[1], RB[1], num_points_line*2)

# 3. RB -> Vertex まで直線
x_line3 = np.linspace(RB[0], Vertex[0], num_points_line)
y_line3 = np.linspace(RB[1], Vertex[1], num_points_line)

# 4. Vertex -> RLM まで直線
x_line4 = np.linspace(Vertex[0], RLM[0], num_points_line*2)
y_line4 = np.linspace(Vertex[1], RLM[1], num_points_line*2)

# 5. RLMから、正三角形の内接円を書いて一周する
# 内接円の中心座標と半径を計算
circle_center_x = x_center
circle_center_y = y_bottom + triangle_height / 3.0
radius = square_size / (2 * np.sqrt(3))

# 角度を生成（-π/2から3π/2まで、一周分）
theta5 = np.linspace(-np.pi / 2, 3 * np.pi / 2, num_points_circle)

# 円の座標を計算
x_line5 = circle_center_x + radius * np.cos(theta5)
y_line5 = circle_center_y + radius * np.sin(theta5)

# パスを連結して一筆書きのパスを生成
x_targets = np.concatenate([
    x_line1, x_line2, x_line3, x_line4, x_line5
])
y_targets = np.concatenate([
    y_line1, y_line2, y_line3, y_line4, y_line5
])

print("length of x_targets: ", len(x_targets))
print("length of y_targets: ", len(y_targets))

# xとyを結合して一つの配列にする
coordinates = np.column_stack((x_targets, y_targets))

# CSVファイルに保存（ヘッダーなし、カンマ区切り）
np.savetxt('triangle.csv', coordinates, delimiter=',')

# 経路をプロットして確認（オプション）
plt.plot(x_targets, y_targets)
plt.axis('equal')
plt.title('Generated Path')
plt.xlabel('X-coordinate (mm)')
plt.ylabel('Y-coordinate (mm)')
plt.grid(True)
plt.show()