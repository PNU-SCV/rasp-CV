import heapq
import math

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_valid_position(bitmap, current, new_point, direction):
    rows, cols = len(bitmap), len(bitmap[0])
    r, c = new_point

    if not (0 <= r < rows and 0 <= c < cols and bitmap[r][c] == 1):
        return False

    if direction in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        r_current, c_current = current
        if not (bitmap[r][c_current] == 1 and bitmap[r_current][c] == 1):
            return False

    return True

def find_shortest_path_astar(bitmap, cur_point, dest_point):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),  
        (-1, -1), (-1, 1), (1, -1), (1, 1)  
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

def real_to_grid(x, z, cell_size_cm=80):
    if math.isnan(x) or math.isnan(z):
        return None
    
    grid_x = int((x * 100) / cell_size_cm) 
    grid_z = int((z * 100) / cell_size_cm) 
    
    return grid_x, grid_z

def grid_to_real(grid_x, grid_z, cell_size_cm=80):
    center_x = (grid_x + 0.5) * cell_size_cm / 100
    center_z = (grid_z + 0.5) * cell_size_cm / 100
    
    return center_x, center_z
