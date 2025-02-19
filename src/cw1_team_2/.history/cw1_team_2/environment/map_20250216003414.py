# 2D Simple Map
import json
import numpy as np

class Environment:
    def __init__(self, map_file="environment/obstacles.json"):
        self.load_map(map_file)

    def load_map(self, map_file):
        """加载障碍物地图"""
        with open(map_file, "r") as f:
            self.obstacles = json.load(f)

    def is_collision(self, pos):
        """检查是否与障碍物碰撞"""
        x, y = pos
        for obs in self.obstacles:
            if obs["x_min"] <= x <= obs["x_max"] and obs["y_min"] <= y <= obs["y_max"]:
                return True
        return False
