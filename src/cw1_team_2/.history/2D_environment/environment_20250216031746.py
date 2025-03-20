import json
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry.point import Point

class Environment:
    """2D 障碍物环境"""

    def __init__(self, map_file=None, robot_radius=0.01):
        self.obstacles = []
        self.robot_radius = robot_radius  # 机器人半径
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
        
        self.obstacles = [Polygon(obs["vertices"]) for obs in data]

    def get_obstacles_near(self, position, sensor_range, resolution=0.1):
        """
        返回在 sensor_range 内的障碍物边界点 (稠密采样)
        :param position: 机器人当前位置
        :param sensor_range: 传感器探测范围
        :param resolution: 采样间隔 (默认 0.1)
        """
        nearby_points = []
        point = Point(position)

        for obs in self.obstacles:
            # 遍历障碍物边界的所有点
            for i in np.arange(0, obs.boundary.length, resolution):
                sampled_point = obs.boundary.interpolate(i)  # 在边界上采样
                if point.distance(sampled_point) <= sensor_range:
                    nearby_points.append((sampled_point.x, sampled_point.y))

        return nearby_points


    def is_collision(self, position):
        """检测机器人当前位置是否与障碍物碰撞"""
        point = Point(position)
        for obs in self.obstacles:
            if obs.contains(point):
                return True
        return False