# 运行 Bug0 测试 (in 2D map)
from cw1_team_2.environment.map import Environment
from cw1_team_2.environment.object_follower import ObjectFollower
from cw1_team_2.utils.visualization import plot_path
from cw1_team_2.utils.config import CONFIG
import numpy as np

class Bug0:
    def __init__(self, env):
        self.env = env
        self.position = np.array([0.0, 0.0])
        self.goal = np.array([4, 4])
        self.path = [tuple(self.position)]
        self.follower = ObjectFollower(env)
        self.closest_point = self.position

    def move(self):
        """模拟 Bug1 运动"""
        while np.linalg.norm(self.goal - self.position) > 0.1:
            if self.env.is_collision(self.position):
                self.position = self.follower.move_around_obstacle()
                if np.linalg.norm(self.goal - self.position) < np.linalg.norm(self.goal - self.closest_point):
                    self.closest_point = self.position
            else:
                direction = self.goal - self.position
                direction = direction / np.linalg.norm(direction) * CONFIG["step_size"]
                self.position += direction
            self.path.append(tuple(self.position))

    def run(self):
        self.move()
        return self.path