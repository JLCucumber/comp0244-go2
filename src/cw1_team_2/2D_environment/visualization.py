import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon

def plot_environment(env, robot):
    """绘制环境、障碍物和机器人轨迹"""

    plt.clf()  # 清除上一帧
    
    # 绘制障碍物
    for obs in env.obstacles:
        if isinstance(obs, Polygon):
            x, y = obs.exterior.xy
            plt.plot(x, y, 'k-', linewidth=2)

    # 绘制机器人轨迹
    path = np.array(robot.path)
    plt.plot(path[:, 0], path[:, 1], 'r-', label="Path", linewidth=2)

    # 绘制机器人当前位置
    plt.scatter(robot.position[0], robot.position[1], color='blue', s=100, edgecolors='black', label="Robot")

    # 绘制机器人朝向
    heading = robot.position + np.array([np.cos(robot.angle), np.sin(robot.angle)]) * 0.2
    plt.plot([robot.position[0], heading[0]], [robot.position[1], heading[1]], 'g-', linewidth=2)

    # 绘制传感器感知到的障碍物点
    local_obstacles = robot.get_local_obstacles()
    if local_obstacles:
        obs_points = np.array(local_obstacles)
        plt.scatter(obs_points[:, 0], obs_points[:, 1], color='red', s=5, label="Detected Obstacles")

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.title("Real-time 2D Robot Visualization with Sensor")
    plt.pause(0.1)
