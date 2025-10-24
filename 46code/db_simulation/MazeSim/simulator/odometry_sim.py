import math

class OdometrySimulator:
    def __init__(self, x=1.5, y=1.5, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def update(self, linear_velocity=0.05, angular_velocity=0.0, dt=1.0):
        """
        模拟更新位置（单位：格子，弧度）
        linear_velocity: 单位为格子/秒
        angular_velocity: 单位为弧度/秒
        """
        self.theta += angular_velocity * dt
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        return self.x, self.y, self.theta

    def get_pose(self):
        return self.x, self.y, self.theta
