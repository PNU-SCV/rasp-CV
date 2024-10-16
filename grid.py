import numpy as np
from utils import real_to_grid 

def create_grid(max_x, max_z, cell_size_cm, obstacles=[]):
    grid_size_x = int(np.ceil(max_x * 100.0 / cell_size_cm))  
    grid_size_z = int(np.ceil(max_z * 100.0 / cell_size_cm))  

    grid = np.ones((grid_size_z, grid_size_x), dtype=int)

    cell_size_m = cell_size_cm / 100.0

    for i in range(grid_size_z):
        for j in range(grid_size_x):
            cell_bottom_right_x = (j + 1) * cell_size_m
            cell_bottom_right_z = (i + 1) * cell_size_m
            if cell_bottom_right_x <= max_x and cell_bottom_right_z <= max_z:
                grid[i, j] = 1  
            else:
                grid[i, j] = 0  

    for obs_x, obs_z in obstacles:
        grid[8 - obs_x, obs_z] = 0
        

    return grid, grid_size_x, grid_size_z

