import numpy as np

class Robot:
    """带方向的可移动机器人"""

    def __init__(self, start_pos=(0.5, 0.5), start_angle=0, step_size=0.1):
        self.position = np.array(start_pos)
        self.angle = start_angle  # 方向，以弧度表示
        self.step_size = step_size  # 每次移动的步长
        self.path = [tuple(self.position)]  # 记录轨迹

    def move_forward(self):
        """沿当前方向前进"""
        dx = self.step_size * np.cos(self.angle)
        dy = self.step_size * np.sin(self.angle)
        self.position += np.array([dx, dy])
        self.path.append(tuple(self.position))

    def rotate(self, delta_angle):
        """旋转机器人"""
        self.angle += delta_angle  # 弧度
