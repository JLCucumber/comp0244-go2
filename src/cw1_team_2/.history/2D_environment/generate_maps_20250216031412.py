import json
import numpy as np
from shapely.geometry import Polygon, Point

def generate_star(center, size):
    """生成星形障碍物"""
    cx, cy = center
    points = []
    for i in range(5):  # 5pi 个角
        angle = np.pi / 5 * i
        radius = size if i % 2 == 0 else size / 2
        x = cx + radius * np.cos(angle)
        y = cy + radius * np.sin(angle)
        points.append((x, y))
    return Polygon(points)

def generate_soft_concave(center, size):
    """生成柔和凹形障碍物（圆角 U 形）"""
    cx, cy = center
    base = Polygon([
        (cx - size, cy - size / 2), (cx + size, cy - size / 2),  # 底部
        (cx + size, cy + size), (cx - size, cy + size)  # 上方两侧
    ])
    return base.buffer(-size / 4)  # 缩小形成凹陷

def save_map_to_json(obstacles, filename):
    """保存障碍物数据到 JSON"""
    obstacle_list = []
    for obs in obstacles:
        coords = list(obs.exterior.coords)
        obstacle_list.append({"vertices": coords})
    
    with open(filename, "w") as f:
        json.dump(obstacle_list, f, indent=4)

if __name__ == "__main__":
    # 生成星形 & 柔和凹形障碍物
    star1 = generate_star(center=(3, 3), size=1)
    star2 = generate_star(center=(6, 6), size=1.5)
    soft_concave1 = generate_soft_concave(center=(3, 7), size=2)

    # 存储到 JSON
    save_map_to_json([star1, star2, soft_concave1], "map_with_complex_obstacles.json")
    print("地图已成功保存为 map_with_complex_obstacles.json")
