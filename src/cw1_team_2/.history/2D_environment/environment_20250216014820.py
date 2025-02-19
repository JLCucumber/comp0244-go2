import numpy as np
import json

class Environment:
    """2D 障碍物环境"""
    
    def __init__(self, map_file=None):
        self.obstacles = []
        if map_file:
            self.load_map(map_file)
        else:
            self.generate_default_map()

    def generate_default_map(self):
        """生成一个默认的障碍物地图"""
        self.obstacles = [
            {"x_min": 1, "y_min": 1, "x_max": 2, "y_max": 3},  # 竖长方形障碍物
            {"x_min": 3, "y_min": 2, "x_max": 4, "y_max": 4},  # 另一个障碍物
        ]

    def load_map(self, map_file):
        """从 JSON 文件加载地图"""
        with open(map_file, "r") as f:
            self.obstacles = json.load(f)

    def get_obstacles_near(self, position, sensor_range):
        """返回在 sensor_range 内的障碍物"""
        nearby_obstacles = []
        for obs in self.obstacles:
            obs_center = np.array([(obs["x_min"] + obs["x_max"]) / 2,
                                   (obs["y_min"] + obs["y_max"]) / 2])
            if np.linalg.norm(position - obs_center) <= sensor_range:
                nearby_obstacles.append(obs)
        return nearby_obstacles

    def is_collision(self, position):
        """检查当前位置是否与障碍物碰撞"""
        x, y = position
        for obs in self.obstacles:
            if obs["x_min"] <= x <= obs["x_max"] and obs["y_min"] <= y <= obs["y_max"]:
                return True
        return False
