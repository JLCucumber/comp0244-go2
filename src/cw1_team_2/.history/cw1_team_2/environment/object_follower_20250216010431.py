import numpy as np

class ObjectFollower:
    """实现逆时针沿障碍物边界运动"""

    def __init__(self, env, follow_distance=0.2, step_size=0.1):
        """
        :param env: 环境对象 (可以是 2D 或 Gazebo)
        :param follow_distance: 机器人保持的障碍物边界距离
        :param step_size: 机器人每次移动的步长
        """
        self.env = env
        self.follow_distance = follow_distance
        self.step_size = step_size
        self.current_pos = np.array([0.0, 0.0])

    def find_nearest_obstacle(self):
        """找到最近的障碍物及其最近边界点"""
        min_dist = float("inf")
        nearest_obs = None
        nearest_edge_point = None

        for obs in self.env.obstacles:
            # 获取障碍物四条边的点
            edge_points = [
                (obs["x_min"], obs["y_min"]),
                (obs["x_max"], obs["y_min"]),
                (obs["x_max"], obs["y_max"]),
                (obs["x_min"], obs["y_max"])
            ]

            for point in edge_points:
                dist = np.linalg.norm(self.current_pos - np.array(point))
                if dist < min_dist:
                    min_dist = dist
                    nearest_obs = obs
                    nearest_edge_point = np.array(point)

        return nearest_obs, nearest_edge_point

    def move_around_obstacle(self):
        """围绕最近的障碍物逆时针运动"""
        nearest_obs, nearest_edge = self.find_nearest_obstacle()

        if nearest_obs is None:
            return self.current_pos  # 没有障碍物

        # 计算当前点到障碍物的方向
        direction = self.current_pos - nearest_edge
        direction = direction / np.linalg.norm(direction)

        # 计算逆时针切向方向 (左手法则)
        tangent_direction = np.array([-direction[1], direction[0]])  # 逆时针旋转 90°
        new_pos = self.current_pos + tangent_direction * self.step_size

        # 更新位置
        self.current_pos = new_pos
        return new_pos
