# grid.py

import numpy as np

# grid.py

import numpy as np
from utils import real_to_grid  # real_to_grid 함수를 가져옵니다.

def create_grid(max_x, max_z, cell_size_cm, obstacles=[]):
    """
    그리드를 생성합니다. 각 셀은 동일한 크기의 정사각형으로 구성되며,
    그리드의 모서리가 (0,0)부터 시작합니다. 그리드 끝 부분이 잘리는 경우 해당 셀은 이동 불가로 설정합니다.
    장애물의 좌표가 주어지면 해당 셀을 장애물로 설정합니다.

    Args:
        max_x (float): x 축 최대 크기 (미터 단위)
        max_z (float): z 축 최대 크기 (미터 단위)
        cell_size_cm (int): 셀 크기 (cm 단위)
        obstacles (list of tuples): 장애물의 실제 좌표 리스트 (예: [(x1, z1), (x2, z2), ...])
    
    Returns:
        np.ndarray: 생성된 그리드 배열
        int: 그리드의 x축 셀 개수
        int: 그리드의 z축 셀 개수
    """
    # 그리드 크기 계산
    grid_size_x = int(np.ceil(max_x * 100.0 / cell_size_cm))  # x축 셀 개수 (올림하여 셀 크기에 맞춤)
    grid_size_z = int(np.ceil(max_z * 100.0 / cell_size_cm))  # z축 셀 개수 (올림하여 셀 크기에 맞춤)

    # 그리드 생성 (기본값 1: 이동 불가)
    grid = np.ones((grid_size_z, grid_size_x), dtype=int)

    # 각 셀의 크기 계산
    cell_size_m = cell_size_cm / 100.0

    # 그리드 채우기 (0: 이동 가능, 1: 이동 불가)
    for i in range(grid_size_z):
        for j in range(grid_size_x):
            cell_bottom_right_x = (j + 1) * cell_size_m
            cell_bottom_right_z = (i + 1) * cell_size_m
            if cell_bottom_right_x <= max_x and cell_bottom_right_z <= max_z:
                grid[i, j] = 1  # 이동 가능
            else:
                grid[i, j] = 0  # 잘린 셀은 이동 불가

    # 장애물 처리
    for obs_x, obs_z in obstacles:
        obs_grid_x, obs_grid_z = real_to_grid(obs_x, obs_z, cell_size_cm)
        if 0 <= obs_grid_x < grid_size_x and 0 <= obs_grid_z < grid_size_z:
            grid[obs_grid_z, obs_grid_x] = 0  # 장애물이 있는 셀은 이동 불가

    return grid, grid_size_x, grid_size_z

