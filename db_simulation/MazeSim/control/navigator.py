import math

class Navigator:
    def __init__(self):
        self.path_index = 0

    def step(self, pose, path):
        """
        给定当前位置和规划路径，输出控制信号（模拟）
        pose: (x, y, theta)
        path: [(x1, y1), (x2, y2), ...]
        返回：线速度, 角速度
        """
        if not path or self.path_index >= len(path):
            return 0.0, 0.0  # 到达终点

        target = path[self.path_index]
        dx = target[0] + 0.5 - pose[0]
        dy = target[1] + 0.5 - pose[1]
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - pose[2]

        # 简化角度差
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) > 0.3:
            return 0.0, 0.5 * angle_diff  # 转向优先

        # 接近目标点则切换下一个
        distance = math.hypot(dx, dy)
        if distance < 0.3:
            self.path_index += 1

        return 0.1, 0.0  # 匀速前进
