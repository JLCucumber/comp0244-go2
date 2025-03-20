# A simple Object Follower in 2D map
import numpy as np

class ObjectFollower:
    def __init__(self, env):
        self.env = env  # 依赖 Environment
        self.current_pos = np.array([0.0, 0.0])

    def find_nearest_obstacle(self):
        """找到最近的障碍物"""
        min_dist = float("inf")
        nearest_obs = None
        for obs in self.env.obstacles:
            obs_center = np.array([(obs["x_min"] + obs["x_max"]) / 2,
                                   (obs["y_min"] + obs["y_max"]) / 2])
            dist = np.linalg.norm(self.current_pos - obs_center)
            if dist < min_dist:
                min_dist = dist
                nearest_obs = obs_center
        return nearest_obs

    def move_around_obstacle(self):
        """简单的绕障逻辑 (逆时针)"""
        obstacle = self.find_nearest_obstacle()
        if obstacle is None:
            return self.current_pos  # 没有障碍物

        direction = np.array([-1, 1])  # 逆时针方向
        return obstacle + direction * 0.1  # 贴近绕行