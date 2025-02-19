import json
import numpy as np

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
            {"x_min": 1, "y_min": 1, "x_max": 2, "y_max": 2},  # 正方形障碍物
            {"x_min": 3, "y_min": 2, "x_max": 4, "y_max": 3},  # 另一个障碍物
        ]

    def load_map(self, map_file):
        """从 JSON 文件加载地图"""
        with open(map_file, "r") as f:
            self.obstacles = json.load(f)

    def get_obstacles_near(self, position, sensor_range):
        """返回在 `sensor_range` 内的障碍物边界点"""
        nearby_points = []
        for obs in self.obstacles:
            # 获取障碍物四个角的坐标
            edge_points = [
                (obs["x_min"], obs["y_min"]),
                (obs["x_max"], obs["y_min"]),
                (obs["x_max"], obs["y_max"]),
                (obs["x_min"], obs["y_max"])
            ]
            
            # 生成额外边界上的点，提高检测精度
            step = 0.02  # 采样间距
            for x in np.arange(obs["x_min"], obs["x_max"], step):
                edge_points.append((x, obs["y_min"]))
                edge_points.append((x, obs["y_max"]))
            for y in np.arange(obs["y_min"], obs["y_max"], step):
                edge_points.append((obs["x_min"], y))
                edge_points.append((obs["x_max"], y))

            for point in edge_points:
                point = np.array(point)
                if np.linalg.norm(position - point) <= sensor_range:
                    nearby_points.append(point)

        return nearby_points

    def is_collision(self, position):
        """检查当前位置是否与障碍物碰撞"""
        x, y = position
        for obs in self.obstacles:
            if obs["x_min"] <= x <= obs["x_max"] and obs["y_min"] <= y <= obs["y_max"]:
                print(f"Collision detected! Robot position: {position}")
                return True

        return False