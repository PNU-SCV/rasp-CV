import heapq
import math

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_valid_position(bitmap, current, new_point, direction):
    rows, cols = len(bitmap), len(bitmap[0])
    r, c = new_point

    # 기본 이동 가능 여부 확인
    if not (0 <= r < rows and 0 <= c < cols and bitmap[r][c] == 1):
        return False

    # 대각선 이동의 경우 추가 검사
    if direction in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        r_current, c_current = current
        if not (bitmap[r][c_current] == 1 and bitmap[r_current][c] == 1):
            return False

    return True

def find_shortest_path_astar(bitmap, cur_point, dest_point):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),   # 상, 하, 좌, 우
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # 대각선
    ]
    rows, cols = len(bitmap), len(bitmap[0])
    open_set = []
    heapq.heappush(open_set, (0, cur_point, [cur_point]))
    visited = set()

    while open_set:
        _, current, path = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)

        if current == dest_point:
            return path

        for d in directions:
            new_row, new_col = current[0] + d[0], current[1] + d[1]
            new_point = (new_row, new_col)

            # 이동 가능 여부를 검사, 방향 정보를 전달
            if new_point not in visited and is_valid_position(bitmap, current, new_point, direction=d):
                new_cost = len(path) + heuristic(new_point, dest_point)
                heapq.heappush(open_set, (new_cost, new_point, path + [new_point]))

    return []

def simplify_path(path):
    if not path:
        return []
    
    simplified_path = [path[0]]  
    
    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        current = path[i]
        next_point = path[i + 1]
        
        direction1 = (current[0] - prev[0], current[1] - prev[1])
        direction2 = (next_point[0] - current[0], next_point[1] - current[1])
        
        if direction1 != direction2:
            simplified_path.append(current)
    
    simplified_path.append(path[-1])
    
    return simplified_path

def real_to_grid(x, z, cell_size_cm=70):
    """
    실제 좌표 (x, z)를 받아 그리드 인덱스 (grid_x, grid_z)로 변환합니다.

    Args:
        x (float): x 좌표 (미터 단위)
        z (float): z 좌표 (미터 단위)
        cell_size_cm (int): 셀 크기 (cm 단위)
    
    Returns:
        tuple: 그리드 인덱스 (grid_x, grid_z)
    """
    if math.isnan(x) or math.isnan(z):
        return None
    
    # Convert x, z to cm and calculate grid index
    grid_x = int((x * 100) / cell_size_cm)  # x * 100 to convert meters to centimeters
    grid_z = int((z * 100) / cell_size_cm)  # z * 100 to convert meters to centimeters
    
    return grid_x, grid_z

def grid_to_real(grid_x, grid_z, cell_size_cm=70):
    """
    그리드 인덱스 (grid_x, grid_z)를 받아 해당 그리드 셀의 중심 실제 좌표 (center_x, center_z)를 반환합니다.

    Args:
        grid_x (int): 그리드의 x 인덱스
        grid_z (int): 그리드의 z 인덱스
        cell_size_cm (int): 셀 크기 (cm 단위)
    
    Returns:
        tuple: 실제 좌표 (center_x, center_z)
    """
    # Calculate the center coordinates of the grid cell
    center_x = (grid_x + 0.5) * cell_size_cm / 100  # Convert to meters
    center_z = (grid_z + 0.5) * cell_size_cm / 100  # Convert to meters
    
    return center_x, center_z