# Given values from the user's files and previous calculations.
resolution = 0.05
origin_xy = (-2.0, 0.0)
img_h = 160
cell_size = 5

# --- Segment 1: (gx=5, gy=22) -> (gx=24, gy=8) ---
# Start point
gx_start_1, gy_start_1 = 5, 22
px_start_1 = gx_start_1 * cell_size
py_start_1 = gy_start_1 * cell_size
x_w_start_1 = px_start_1 * resolution + origin_xy[0]
y_w_start_1 = (img_h - 1 - py_start_1) * resolution + origin_xy[1]

# Goal point
gx_goal_1, gy_goal_1 = 24, 7
px_goal_1 = gx_goal_1 * cell_size
py_goal_1 = gy_goal_1 * cell_size
x_w_goal_1 = px_goal_1 * resolution + origin_xy[0]
y_w_goal_1 = (img_h - 1 - py_goal_1) * resolution + origin_xy[1]

print(f"Segment 1:")
print(f"Start World: ({x_w_start_1}, {y_w_start_1})")
print(f"Goal World: ({x_w_goal_1}, {y_w_goal_1})")

# --- Segment 2: (gx=24, gy=8) -> (gx=5, gy=11) ---
# Start point (same as goal of segment 1)
x_w_start_2, y_w_start_2 = x_w_goal_1, y_w_goal_1

# Goal point
gx_goal_2, gy_goal_2 = 5, 11
px_goal_2 = gx_goal_2 * cell_size
py_goal_2 = gy_goal_2 * cell_size
x_w_goal_2 = px_goal_2 * resolution + origin_xy[0]
y_w_goal_2 = (img_h - 1 - py_goal_2) * resolution + origin_xy[1]

print(f"\nSegment 2:")
print(f"Start World: ({x_w_start_2}, {y_w_start_2})")
print(f"Goal World: ({x_w_goal_2}, {y_w_goal_2})")