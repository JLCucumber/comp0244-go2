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

    def check_if_near_obstacle(self):
        """检查机器人是否接近障碍物"""
        local_obstacles = self.robot.get_local_obstacles()
        if not local_obstacles:
            return None  # 没有障碍物
        
        # 找到最近的障碍物点
        min_dist = float("inf")
        nearest_obs_point = None
        for point in local_obstacles:
            dist = np.linalg.norm(self.robot.position - point)
            if dist < min_dist:
                min_dist = dist
                nearest_obs_point = np.array(point)

        # 如果距离小于设定的 `follow_distance`，启用绕障模式
        if min_dist <= self.follow_distance:
            self.following_obstacle = True
            return nearest_obs_point
        else:
            self.following_obstacle = False
            return None

    def follow_obstacle(self):
        """执行逆时针绕障行为"""
        if not self.following_obstacle:
            print("Robot not in obstacle following mode.")
            return False  # 机器人未进入绕障模式

        nearest_obs = self.check_if_near_obstacle()
        if nearest_obs is None:
            self.following_obstacle = False
            print("No obstacle detected, stop following.")
            return False  # 没有检测到障碍物

        # 计算到障碍物的方向（法向量）
        direction = self.robot.position - nearest_obs
        direction /= np.linalg.norm(direction)

        # 计算逆时针切向方向 (左手法则，旋转 90°)
        tangent_direction = np.array([-direction[1], direction[0]])

        # 让机器人沿着障碍物边界移动
        self.robot.position += tangent_direction * self.step_size
        self.robot.path.append(tuple(self.robot.position))

        print("Detected obstacle, following it...")
        # print(f"Current position: {self.robot.position}")
        print(f"next position: {self.robot.position + tangent_direction * self.step_size}")

        return True
