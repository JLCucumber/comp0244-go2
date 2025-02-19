import numpy as np

class ObstacleFollower:
    """机器人绕障碍物逆时针运动"""

    def __init__(self, robot, follow_distance=0.2, step_size=0.05):
        """
        :param robot: 机器人对象
        :param follow_distance: 机器人希望保持的障碍物边界距离
        :param step_size: 机器人每次移动的步长
        """
        self.robot = robot
        self.follow_distance = follow_distance
        self.step_size = step_size
        self.following_obstacle = False  # 是否在绕障模式
        self.last_tangent_direction = None  # 记录上一步的切向方向

    def check_if_near_obstacle(self):
        """检查机器人是否接近障碍物，并计算最近的边界点"""
        local_obstacles = self.robot.get_local_obstacles()
        if not local_obstacles:
            return None, float("inf")  # 没有障碍物
        
        # 找到最近的障碍物点
        min_dist = float("inf")
        nearest_obs_point = None
        for point in local_obstacles:
            dist = np.linalg.norm(self.robot.position - point)
            if dist < min_dist:
                min_dist = dist
                nearest_obs_point = np.array(point)

        return nearest_obs_point, min_dist

    def follow_obstacle(self):
        """执行逆时针绕障行为，保持机器人贴近障碍物"""
        nearest_obs, nearest_obs_dist = self.check_if_near_obstacle()

        if nearest_obs is None:
            # 如果短暂丢失障碍物检测，继续沿着上次方向移动，避免脱离
            if self.last_tangent_direction is not None:
                self.robot.position += self.last_tangent_direction * self.step_size
                self.robot.path.append(tuple(self.robot.position))
                return True
            else:
                self.following_obstacle = False
                return False  # 退出绕障模式

        # 计算到障碍物的方向（法向量）
        direction = self.robot.position - nearest_obs
        direction /= np.linalg.norm(direction)

        # 计算逆时针切向方向 (左手法则，旋转 90°)
        tangent_direction = np.array([-direction[1], direction[0]])

        # 平滑拐角调整（防止剧烈方向变化）
        if self.last_tangent_direction is not None:
            tangent_direction = 0.7 * self.last_tangent_direction + 0.3 * tangent_direction
            tangent_direction /= np.linalg.norm(tangent_direction)

        # **🚀 关键改进：调整机器人位置，使其保持恒定距离**
        distance_error = nearest_obs_dist - self.follow_distance
        correction_vector = -direction * distance_error * 0.5  # 控制机器人靠近/远离障碍物

        # 让机器人沿着障碍物边界移动，同时修正靠近障碍物
        self.robot.position += tangent_direction * self.step_size + correction_vector
        self.robot.path.append(tuple(self.robot.position))

        # 记录最新的切向方向
        self.last_tangent_direction = tangent_direction
        return True