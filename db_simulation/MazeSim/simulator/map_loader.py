import json
import numpy as np

def load_map(json_path):
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    segments = data['segments']

    # 自动推断尺寸
    max_x = max(max(seg['start'][0], seg['end'][0]) for seg in segments)
    max_y = max(max(seg['start'][1], seg['end'][1]) for seg in segments)
    size_x = max_x + 1
    size_y = max_y + 1
    maze = np.zeros((size_y, size_x), dtype=int)  # shape=(y,x)

    for seg in segments:
        x1, y1 = seg['start']
        x2, y2 = seg['end']
        if x1 == x2:
            # 竖线
            for y in range(min(y1, y2), max(y1, y2) + 1):
                maze[y, x1] = 1
        elif y1 == y2:
            # 横线
            for x in range(min(x1, x2), max(x1, x2) + 1):
                maze[y1, x] = 1
        else:
            raise ValueError("线段必须是水平或垂直的")

    # 互换横纵坐标（xy -> yx）
    raw_start = data.get('start_point', (1, 0))
    start_point = (raw_start[1], raw_start[0])
    if 'end_point' in data:
        raw_end = data['end_point']
        end_point = (raw_end[1], raw_end[0])
    else:
        end_point = None

    return maze, start_point, end_point
