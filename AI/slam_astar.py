import cv2
import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Any

# =========================
# 1) 맵 로드 & 전처리
# =========================
def load_map(path: str) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Map image not found: {path}")
    return img

def classify_map(image: np.ndarray) -> np.ndarray:
    """
    occupancy 분류:
      0~50   : 장애물(2)
      200~255: 자유(1)
      나머지: 미확인(0)
    """
    grid = np.zeros_like(image, dtype=np.uint8)
    grid[image <= 50] = 2
    grid[image >= 200] = 1
    grid[(image > 50) & (image < 200)] = 0
    return grid

def generate_grid(occ_grid_img: np.ndarray, map_res: float, robot_size_m: float):
    """
    픽셀 격자를 로봇 크기 기반 셀 격자로 다운샘플
    - cell_size(px) = robot_size / (2 * resolution)
    - 최소 1 보장
    """
    cell_size = max(1, int(round(robot_size_m / (2 * map_res))))
    h, w = occ_grid_img.shape
    new_h, new_w = h // cell_size, w // cell_size
    new_grid = np.zeros((new_h, new_w), dtype=np.uint8)

    for gy in range(new_h):
        for gx in range(new_w):
            y1, y2 = gy*cell_size, (gy+1)*cell_size
            x1, x2 = gx*cell_size, (gx+1)*cell_size
            cell = occ_grid_img[y1:y2, x1:x2]
            if np.any(cell == 2):
                new_grid[gy, gx] = 2
            elif np.all(cell == 1):
                new_grid[gy, gx] = 1
            else:
                new_grid[gy, gx] = 0
    return new_grid, cell_size

# =========================
# 2) 좌표 변환 (ROS world <-> image/grid)
# =========================
def world_to_pixel(x_w: float, y_w: float, origin_xy: Tuple[float,float],
                   resolution: float, img_h: int):
    """
    ROS world -> image pixel
    - origin_xy=(x0,y0): 이미지 '좌하단'의 world 좌표 (map.yaml origin[0:2])
    - resolution: m/pixel
    - img_h: 이미지 높이(픽셀)
    이미지 좌표계는 (row=0)이 맨 위이므로 y를 뒤집는다.
    """
    x0, y0 = origin_xy
    px = (x_w - x0) / resolution
    py_world_up = (y_w - y0) / resolution
    py = img_h - 1 - py_world_up
    return px, py

def pixel_to_world(px: float, py: float, origin_xy: Tuple[float,float],
                   resolution: float, img_h: int):
    x0, y0 = origin_xy
    x_w = px * resolution + x0
    y_w = (img_h - 1 - py) * resolution + y0
    return x_w, y_w

def world_to_grid(x_w: float, y_w: float, origin_xy: Tuple[float,float],
                  resolution: float, img_h: int, cell_size: int):
    """
    ROS world -> grid index (gy, gx)
    1) world -> pixel
    2) pixel -> grid (셀 크기로 나눔)
    """
    px, py = world_to_pixel(x_w, y_w, origin_xy, resolution, img_h)
    gx = int(px // cell_size)
    gy = int(py // cell_size)
    return gy, gx  # (행, 열)

# =========================
# 3) A* (4-이웃)
# =========================
def a_star(grid: np.ndarray, start: Tuple[int,int], goal: Tuple[int,int]):
    h, w = grid.shape
    
    # 장애물 확장
    expanded_grid = grid.copy()
    
    # 로봇 크기 절반만큼 확장 (cell_size가 로봇 크기 절반이므로 1칸 확장)
    expansion_distance = 1 

    # 주변 8개 방향을 확인
    neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
    
    for gy in range(h):
        for gx in range(w):
            if grid[gy, gx] == 2:  # 장애물 셀을 발견하면
                for dy, dx in neighbors:
                    ny, nx = gy + dy, gx + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        if expanded_grid[ny, nx] == 1: # 자유 공간인 경우에만 확장
                            expanded_grid[ny, nx] = 2
                            
    def in_bounds(ny, nx): return 0 <= ny < h and 0 <= nx < w
    def heuristic(a, b):  return abs(a[0]-b[0]) + abs(a[1]-b[1])

    directions = [(0,1),(1,0),(0,-1),(-1,0)]  # (dy, dx)
    visited_costs = {}   # node -> (g, turns)
    came_from = {}

    pq = [(heuristic(start, goal), 0, 0, start, None)]
    visited_costs[start] = (0, 0)

    while pq:
        f, turns, g, cur, prev = heapq.heappop(pq)
        if (g, turns) > visited_costs.get(cur, (1e9, 1e9)):
            continue
        if cur == goal:
            break

        for dx, dy in [(1,0),(0,1),(-1,0),(0,-1)]:
            ny, nx = cur[0] + dy, cur[1] + dx
            nb = (ny, nx)
            if not in_bounds(ny, nx):        continue
            if expanded_grid[ny, nx] != 1:              continue  # 자유칸만
            ng = g + 1
            nturns = turns
            if prev is not None:
                prev_dir = (cur[0]-prev[0], cur[1]-prev[1])
                curr_dir = (dy, dx)
                if prev_dir != curr_dir:
                    nturns += 1
            info = (ng, nturns)
            if info < visited_costs.get(nb, (1e9, 1e9)):
                visited_costs[nb] = info
                came_from[nb] = cur
                nf = ng + heuristic(nb, goal)
                heapq.heappush(pq, (nf, nturns, ng, nb, cur))

    # 경로 복원
    path = []
    node = goal
    if node not in came_from and node != start:
        return []  # 경로 없음
    while node in came_from:
        path.append(node)
        node = came_from[node]
    path.append(start)
    path.reverse()
    return path

# =========================
# 4) 명령 생성 (셀 중심 X, 시작 월드좌표에서 적분)
# =========================
# 방향 인코딩: E(0), N(1), W(2), S(3)
def dir_from_step(prev: Tuple[int,int], curr: Tuple[int,int]) -> int:
    py, px = prev
    cy, cx = curr
    dy, dx = cy - py, cx - px
    if dx ==  1 and dy ==  0: return 0  # E
    if dx ==  0 and dy == -1: return 1  # N (grid에서 위로 가면 gy가 감소)
    if dx == -1 and dy ==  0: return 2  # W
    if dx ==  0 and dy ==  1: return 3  # S
    raise ValueError("Non 4-neighbor step in path")

def left_or_right(curr_dir: int, next_dir: int) -> str:
    delta = (next_dir - curr_dir) % 4
    if delta == 1:  return "TURN_LEFT"
    if delta == 3:  return "TURN_RIGHT"
    if delta == 2:  return "TURN_LEFT_LEFT"  # 180° 예외 처리
    return "NONE"

def quantize_heading_to_cardinal(theta: float) -> int:
    deg = (math.degrees(theta) + 360.0) % 360.0
    bins = [0, 90, 180, 270]  # E,N,W,S
    return min(range(4), key=lambda i: abs(deg - bins[i]))

def plan_commands_with_world_NO_CENTER(path: List[Tuple[int,int]],
                                       start_world_xy: Tuple[float,float],
                                       start_heading_rad: float,
                                       resolution: float,
                                       cell_size: int) -> List[Dict[str, Any]]:
    """
    셀 '중심'을 전혀 쓰지 않고, 실제 시작 좌표에서 run-length를 누적해
    월드 좌표를 계산합니다.
    """
    if len(path) < 2:
        sx, sy = start_world_xy
        return [{"cmd":"STOP", "at":(sx, sy)}]

    dirs = [dir_from_step(path[i], path[i+1]) for i in range(len(path)-1)]

    runs: List[Tuple[int,int]] = []
    cur_dir = dirs[0]
    run_len = 1
    for i in range(1, len(dirs)):
        if dirs[i] == cur_dir:
            run_len += 1
        else:
            runs.append((cur_dir, run_len))
            cur_dir = dirs[i]
            run_len = 1
    runs.append((cur_dir, run_len))

    cmds: List[Dict[str, Any]] = []
    sx, sy = start_world_xy
    x, y = sx, sy
    heading_dir = quantize_heading_to_cardinal(start_heading_rad)
    cell_size_m = resolution * cell_size
    
    park_length = 3

    first_dir, _ = runs[0]
    if first_dir != heading_dir:
        cmds.append({"cmd":"STOP", "at": (x, y)})
        turn = left_or_right(heading_dir, first_dir)
        if turn == "TURN_LEFT_LEFT":
            cmds.append({"cmd":"TURN_LEFT", "value":90.0, "at": (x, y)})
            cmds.append({"cmd":"TURN_LEFT", "value":90.0, "at": (x, y)})
        elif turn != "NONE":
            cmds.append({"cmd":turn, "value":90.0, "at": (x, y)})
        heading_dir = first_dir

    DIR_VEC = {
        0: ( 1.0,  0.0),  # E
        1: ( 0.0,  1.0),  # N
        2: (-1.0,  0.0),  # W
        3: ( 0.0, -1.0),  # S
    }

    for i, (d, steps) in enumerate(runs):
        dist = steps * cell_size_m
        vx, vy = DIR_VEC[d]
        
        if i == len(runs) - 1:
            if steps > park_length:
                forward_dist = (steps - park_length) * cell_size_m
                x_next = x + vx * forward_dist
                y_next = y + vy * forward_dist
                cmds.append({"cmd":"FORWARD", "value": forward_dist, "start": (x, y), "end": (x_next, y_next)})
                
                x, y = x_next, y_next
                dist = park_length * cell_size_m
            
            cmds.append({"cmd": "STOP", "at": (x, y)})
            cmds.append({"cmd": "TURN_LEFT", "value": 90.0, "at": (x, y)})
            cmds.append({"cmd": "TURN_LEFT", "value": 90.0, "at": (x, y)})
            
            x_next = x + DIR_VEC[(d + 2) % 4][0] * dist
            y_next = y + DIR_VEC[(d + 2) % 4][1] * dist
            
            cmds.append({"cmd": "BACKWARD", "value": dist, "start": (x, y), "end": (x_next, y_next)})
            
            x, y = x_next, y_next
            cmds.append({"cmd": "STOP", "at": (x, y)})
            return cmds
        
        x_next = x + vx * dist
        y_next = y + vy * dist

        cmds.append({"cmd":"FORWARD", "value": dist, "start": (x, y), "end": (x_next, y_next)})

        if i < len(runs)-1:
            corner = (x_next, y_next)
            cmds.append({"cmd":"STOP", "at": corner})
            d_next = runs[i+1][0]
            turn = left_or_right(d, d_next)
            if turn == "TURN_LEFT_LEFT":
                cmds.append({"cmd":"TURN_LEFT", "value":90.0, "at": corner})
                cmds.append({"cmd":"TURN_LEFT", "value":90.0, "at": corner})
            else:
                cmds.append({"cmd":turn, "value":90.0, "at": corner})

        x, y = x_next, y_next
        heading_dir = d

    cmds.append({"cmd":"STOP", "at": (x, y)})
    return cmds

# =========================
# 5) 시각화
# =========================
def visualize_grid_and_path(grid: np.ndarray, path=None):
    """
    grid: 0(미확인)=회색, 1(자유)=흰색, 2(장애물)=검정
    path: [(gy,gx), ...]
    """
    img = np.zeros_like(grid, dtype=np.uint8)
    img[grid == 1] = 255
    img[grid == 2] = 0
    img[grid == 0] = 150

    if path:
        for (gy, gx) in path:
            if 0 <= gy < img.shape[0] and 0 <= gx < img.shape[1]:
                img[gy, gx] = 128

    plt.figure()
    plt.imshow(img, cmap='gray', interpolation='nearest')
    plt.title("Grid Map with A* Path")
    plt.xlabel("gx")
    plt.ylabel("gy (top→down)")
    plt.show()

# =========================
# 실행 예시
# =========================
if __name__ == "__main__":
    # === 맵/파라미터 ===
    map_file      = "slam_tuning/final_map.pgm"
    resolution    = 0.05
    origin_xy     = (-2.0, 0)
    robot_size    = 0.5

    # === 맵 로드 및 전처리 ===
    original_map = load_map(map_file)
    img_h, img_w = original_map.shape
    occ_img      = classify_map(original_map)
    grid, cell_size = generate_grid(occ_img, resolution, robot_size)
    print(f"[INFO] image={img_w}x{img_h}, cell_size(px)={cell_size}, grid={grid.shape}")

    # === 시작/목표 (world 좌표, 단위 m) ===
    # 그리드 좌표 (gx=2.5, gy=11) 및 (gx=2.5, gy=5)에 맞게 재계산
    start_world = (4.0, 6.2)
    goal_world  = (-0.75, 5.2)
    start_theta = -1.57

    # === world -> grid index ===
    start_idx = world_to_grid(start_world[0], start_world[1], origin_xy, resolution, img_h, cell_size)
    goal_idx  = world_to_grid(goal_world[0],  goal_world[1],  origin_xy, resolution, img_h, cell_size)
    print(f"[INFO] start(grid)={start_idx}, goal(grid)={goal_idx}")

    # === 범위/가용성 체크 ===
    H, W = grid.shape
    for name, (gy, gx) in [("start", start_idx), ("goal", goal_idx)]:
        if not (0 <= gy < H and 0 <= gx < W):
            raise ValueError(f"{name} index {gy,gx} out of grid bounds {H,W}")
        if grid[gy, gx] != 1:
            print(f"[WARN] {name} cell is not free (grid gy={gy}, gx={gx}). A* may fail.")

    # === A* ===
    path = a_star(grid, start_idx, goal_idx)
    if not path:
        print("[ERROR] 경로를 찾지 못했습니다.")
    else:
        print(f"[INFO] path length: {len(path)}")

    # === 명령 생성 (셀 중심 X, 시작 월드에서 적분) ===
    cmds = plan_commands_with_world_NO_CENTER(
        path,
        start_world_xy=start_world,
        start_heading_rad=start_theta,
        resolution=resolution,
        cell_size=cell_size,
    )

    print("\n[COMMANDS]")
    for c in cmds:
        print(c)

    # === 시각화 ===
    visualize_grid_and_path(grid, path)