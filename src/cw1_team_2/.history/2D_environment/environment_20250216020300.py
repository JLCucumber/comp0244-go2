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
        """生成默认障碍物地图"""
        self.obstacles = [
            {"x_min": 1, "y_min": 1, "x_max": 2, "y_max": 2},  # 方形障碍物
            {"x_min": 3, "y_min": 2, "x_max": 4, "y_max": 3},  # 另一个障碍物
        ]

    def load_map(self, map_file):
        """从 JSON 文件加载地图"""
        with open(map_file, "r") as f:
            self.obstacles = json.load(f)
