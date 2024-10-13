import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# System Parameters
l0 = 100.0  # Distance between motors (mm)
l1 = 75.0  # Length of first link (mm)
l2 = 50.0  # Length of second link (mm)

# PID Control Parameters
Kp = 10.0
Ki = 0.0
Kd = 0.2

# Control Interval
control_interval = 0.01  # seconds

# Encoder Settings
CPR = 1000        # Counts per revolution
GearRatio = 1.0   # Gear ratio

# PID Control Variables for Left Arm
integral1_left = 0.0
prevError1_left = 0.0
integral2_left = 0.0
prevError2_left = 0.0

# PID Control Variables for Right Arm
integral1_right = 0.0
prevError1_right = 0.0
integral2_right = 0.0
prevError2_right = 0.0

# Define a Smaller and Simpler Path (Square)
square_size = 10.0  # Half side length (mm), making the full square 50mm x 50mm
num_points_per_side = 100
square_angles = [0, np.pi/2, np.pi, 3*np.pi/2]

# Center the square so that both arms can reach
square_center_x = l0 / 2  # 100 mm
square_center_y = 100.0

# Generate Square Path Points
path_points = []
for angle in square_angles:
    x = square_size * np.cos(angle) + square_center_x
    y = square_size * np.sin(angle) + square_center_y
    path_points.append((x, y))

# Interpolate Points Between Square Corners
x_targets = []
y_targets = []
for i in range(len(path_points)):
    start = path_points[i]
    end = path_points[(i + 1) % len(path_points)]
    x_side = np.linspace(start[0], end[0], num_points_per_side, endpoint=False)
    y_side = np.linspace(start[1], end[1], num_points_per_side, endpoint=False)
    x_targets.extend(x_side)
    y_targets.extend(y_side)

# Convert to NumPy Arrays
x_targets = np.array(x_targets)
y_targets = np.array(y_targets)

# Helper Function to Calculate Minimal Angle Difference
def angle_difference(a, b):
    """
    Calculate the minimal difference between two angles a and b.
    The result is between -pi and pi.
    """
    diff = a - b
    return np.arctan2(np.sin(diff), np.cos(diff))

# Inverse Kinematics for Left Arm with Solution Selection
def inverse_kinematics_left(x_p, y_p, theta1_current, theta2_current):
    D = (x_p**2 + y_p**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1.0:
        return None, None
    # Possible solutions for theta2
    theta2_options = [np.arctan2(np.sqrt(1 - D**2), D),
                      np.arctan2(-np.sqrt(1 - D**2), D)]
    solutions = []
    for theta2 in theta2_options:
        theta1 = np.arctan2(y_p, x_p) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
        solutions.append((theta1, theta2))
    # Select solution closest to current angles
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

# Inverse Kinematics for Right Arm with Solution Selection
def inverse_kinematics_right(x_p, y_p, theta1_current, theta2_current):
    # Right motor is at (l0, 0)
    x = x_p - l0
    y = y_p
    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1.0:
        return None, None
    # Possible solutions for theta2
    theta2_options = [np.arctan2(np.sqrt(1 - D**2), D),
                      np.arctan2(-np.sqrt(1 - D**2), D)]
    solutions = []
    for theta2 in theta2_options:
        theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
        solutions.append((theta1, theta2))
    # Select solution closest to current angles
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

# PID Control Implementation
def pid_control(setpoint, measured, integral, prev_error, dt):
    error = setpoint - measured
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    output = Kp * error + Ki * integral + Kd * derivative
    prev_error = error
    return output, integral, prev_error

# Simulation Initialization
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

# Initial Position
x_p0 = x_targets[0]
y_p0 = y_targets[0]

# Initial Angles (Assuming starting from current angles)
theta1_current_left = 0.0
theta2_current_left = 180.0
theta1_current_right = 0.0
theta2_current_right = 0.0

# Calculate Initial Joint Angles
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

# Initialize Encoder Counts (assuming starting angles are the current angles)
leftEncoderCount = (theta1_left / (2 * np.pi)) * CPR * GearRatio
rightEncoderCount = (theta1_right / (2 * np.pi)) * CPR * GearRatio

# Update Current Angles
theta1_current_left = theta1_left
theta2_current_left = theta2_left
theta1_current_right = theta1_right
theta2_current_right = theta2_right

t = 0.0

# Lists to store the trail positions
trail_x = []
trail_y = []

for x_target, y_target in zip(x_targets, y_targets):
    # Inverse Kinematics for Both Arms
    result_left = inverse_kinematics_left(x_target, y_target, theta1_current_left, theta2_current_left)
    result_right = inverse_kinematics_right(x_target, y_target, theta1_current_right, theta2_current_right)
    
    if result_left is None or result_right is None:
        print(f"At time {t:.2f}s, inverse kinematics solution not found for position ({x_target:.2f}, {y_target:.2f})")
        # Maintain previous angles
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
    
    # PID Control for Left Arm
    controlSignal1, integral1_left, prevError1_left = pid_control(
        theta1_target_left, theta1_current_left, integral1_left, prevError1_left, control_interval)
    controlSignal2, integral2_left, prevError2_left = pid_control(
        theta2_target_left, theta2_current_left, integral2_left, prevError2_left, control_interval)
    
    # PID Control for Right Arm
    controlSignal3, integral1_right, prevError1_right = pid_control(
        theta1_target_right, theta1_current_right, integral1_right, prevError1_right, control_interval)
    controlSignal4, integral2_right, prevError2_right = pid_control(
        theta2_target_right, theta2_current_right, integral2_right, prevError2_right, control_interval)
    
    # Update Current Angles
    theta1_current_left += controlSignal1 * control_interval
    theta2_current_left += controlSignal2 * control_interval
    theta1_current_right += controlSignal3 * control_interval
    theta2_current_right += controlSignal4 * control_interval
    
    # Save Angles and Positions
    theta1_left_list.append(np.degrees(theta1_current_left))
    theta2_left_list.append(np.degrees(theta2_current_left))
    theta1_right_list.append(np.degrees(theta1_current_right))
    theta2_right_list.append(np.degrees(theta2_current_right))
    
    # Calculate Arm End Positions
    # Left Arm
    x1 = l1 * np.cos(theta1_current_left)
    y1 = l1 * np.sin(theta1_current_left)
    x_p_left = x1 + l2 * np.cos(theta1_current_left + theta2_current_left)
    y_p_left = y1 + l2 * np.sin(theta1_current_left + theta2_current_left)
    
    # Right Arm
    x2 = l0 + l1 * np.cos(theta1_current_right)
    y2 = l1 * np.sin(theta1_current_right)
    x_p_right = x2 + l2 * np.cos(theta1_current_right + theta2_current_right)
    y_p_right = y2 + l2 * np.sin(theta1_current_right + theta2_current_right)
    
    # Ensure Both Arms Reach the Same Pen Position by Averaging
    x_p = (x_p_left + x_p_right) / 2
    y_p = (y_p_left + y_p_right) / 2
    
    # Save Positions
    x_p_list.append(x_p)
    y_p_list.append(y_p)
    x1_list.append(x1)
    y1_list.append(y1)
    x2_list.append(x2)
    y2_list.append(y2)
    
    # Append current pen position to trail
    trail_x.append(x_p)
    trail_y.append(y_p)
    
    # Save Time
    time_list.append(t)
    t += control_interval

# Set Up Animation
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-50, l0 + l1 + l2 + 50)
ax.set_ylim(-l1 - l2 - 50, l1 + l2 + 50)
ax.set_xlabel('X Position (mm)')
ax.set_ylabel('Y Position (mm)')
ax.set_title('Robot Arm Movement Animation with Pen Trajectory')
ax.grid()
ax.axis('equal')

# Initialize Plot Elements
line_left_arm, = ax.plot([], [], 'b-o', linewidth=2, label='Left Arm')
line_right_arm, = ax.plot([], [], 'g-o', linewidth=2, label='Right Arm')
pen_point, = ax.plot([], [], 'ro', markersize=6, label='Pen')
pen_trail, = ax.plot([], [], 'r-', linewidth=1, alpha=0.5, label='Pen Trajectory')  # 軌跡用のライン
ax.legend()

# Animation Function
def animate_func(i):
    if i >= len(x_p_list):
        i = len(x_p_list) - 1
    
    # Current Pen Position
    x_p = x_p_list[i]
    y_p = y_p_list[i]
    
    # Left Arm Coordinates
    x_left = [0, x1_list[i], x_p]
    y_left = [0, y1_list[i], y_p]
    
    # Right Arm Coordinates
    x_right = [l0, x2_list[i], x_p]
    y_right = [0, y2_list[i], y_p]
    
    # Update Plot Data
    line_left_arm.set_data(x_left, y_left)
    line_right_arm.set_data(x_right, y_right)
    pen_point.set_data([x_p], [y_p])  # Pass as lists
    
    # Update Pen Trajectory
    pen_trail.set_data(trail_x[:i+1], trail_y[:i+1])
    
    return line_left_arm, line_right_arm, pen_point, pen_trail

# Create Animation
ani = FuncAnimation(fig, animate_func, frames=len(x_p_list), interval=control_interval*1000, blit=True)

plt.show()

# Plot Joint Angles Over Time
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