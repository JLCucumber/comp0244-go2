import numpy as np

class ObstacleFollower:
    """用于计算绕障路径的类"""

    def __init__(self, follow_distance=0.3, step_size=0.1):
        """
        :param follow_distance: 希望保持的障碍物边界距离
        :param step_size: 每次移动的步长
        """
        self.follow_distance = follow_distance
        self.step_size = step_size
        self.last_tangent_direction = None  # 记录上次的切向方向

    def compute_next_position(self, current_position, local_obstacles):
        """
        计算绕障新位置
        :param current_position: (x, y) 机器人当前位置
        :param local_obstacles: [(x1, y1), (x2, y2), ...] 障碍物点列表
        :return: (new_x, new_y) 机器人下一个目标位置
        """
        if not local_obstacles:
            return current_position  # 没有障碍物，不调整路径

        # 计算最近的障碍物点
        min_dist = float("inf")
        nearest_obs_point = None
        for point in local_obstacles:
            dist = np.linalg.norm(np.array(current_position) - np.array(point))
            if dist < min_dist:
                min_dist = dist
                nearest_obs_point = np.array(point)

        if nearest_obs_point is None:
            return current_position

        # 计算到障碍物的方向（法向量）
        direction = np.array(current_position) - nearest_obs_point
        direction /= np.linalg.norm(direction)

        # 计算切向方向 (左手法则，逆时针)
        tangent_direction = np.array([-direction[1], direction[0]])

        # 平滑处理，避免剧烈方向变化
        if self.last_tangent_direction is not None:
            tangent_direction = 0.7 * self.last_tangent_direction + 0.3 * tangent_direction
            tangent_direction /= np.linalg.norm(tangent_direction)

        # 计算修正向量，保持恒定距离
        distance_error = min_dist - self.follow_distance
        correction_vector = -direction * distance_error * 0.5  # 纠正到理想距离

        # 计算新的机器人目标位置
        new_position = np.array(current_position) + tangent_direction * self.step_size + correction_vector

        # 更新切向方向
        self.last_tangent_direction = tangent_direction
        return tuple(new_position)
