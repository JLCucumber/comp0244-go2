import matplotlib.pyplot as plt
import numpy as np

def plot_environment(env, robot):
    """绘制环境、障碍物和机器人位置"""
    plt.clf()  # 清除上一帧

    # 绘制障碍物
    for obs in env.obstacles:
        width = abs(obs["x_max"] - obs["x_min"])
        height = abs(obs["y_max"] - obs["y_min"])
        plt.gca().add_patch(plt.Rectangle((obs["x_min"], obs["y_min"]), width, height, color="black"))

    # 绘制机器人轨迹
    path = np.array(robot.path)
    plt.plot(path[:, 0], path[:, 1], 'r-', label="Path", linewidth=2)

    # 绘制机器人当前位置
    plt.scatter(robot.position[0], robot.position[1], color='blue', s=100, edgecolors='black', label="Robot")

    # 绘制机器人朝向
    heading = robot.position + np.array([np.cos(robot.angle), np.sin(robot.angle)]) * 0.2
    plt.plot([robot.position[0], heading[0]], [robot.position[1], heading[1]], 'g-', linewidth=2)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.title("Real-time 2D Robot Visualization")
    plt.pause(0.1)
