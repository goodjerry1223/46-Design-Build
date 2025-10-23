import numpy as np

def simulate_lidar(pos, maze, scan_size=32, max_dist=6):
    """从当前pos 360°每隔step角度发射一根射线，返回所有能看到的格子"""
    visible = set()
    x, y = pos
    h, w = maze.shape
    angles = np.linspace(0, 2*np.pi, scan_size, endpoint=False)
    for theta in angles:
        for d in range(1, max_dist+1):
            nx = int(round(x + d*np.cos(theta)))
            ny = int(round(y + d*np.sin(theta)))
            if 0 <= nx < h and 0 <= ny < w:
                visible.add((nx, ny))
                if maze[nx, ny] == 1:
                    break
            else:
                break
    return visible

def simulate_lidar_with_distances(pos, maze, scan_size=32, max_dist=6):
    """从当前pos 360°每隔step角度发射一根射线，返回可见格子和距离数据"""
    visible = set()
    x, y = pos
    h, w = maze.shape
    angles = np.linspace(0, 2*np.pi, scan_size, endpoint=False)
    distances = []
    
    for theta in angles:
        hit_distance = max_dist
        for d in range(1, max_dist+1):
            nx = int(round(x + d*np.cos(theta)))
            ny = int(round(y + d*np.sin(theta)))
            if 0 <= nx < h and 0 <= ny < w:
                visible.add((nx, ny))
                if maze[nx, ny] == 1:
                    hit_distance = d
                    break
            else:
                hit_distance = d - 1 if d > 1 else 0
                break
        distances.append(hit_distance)
    
    return visible, angles, distances
