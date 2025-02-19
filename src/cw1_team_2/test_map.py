# from cw1_team_2.environment.map import Environment
# from cw1_team_2.utils.visualization import plot_path
# from cw1_team_2.utils.config import CONFIG

# # if __name__ == "__main__":
# #     env = Environment(CONFIG["map_file"])
# #     start = (0, 0)
# #     goal = (4, 4)
# #     path = [start, goal]  # Dummy path for visualization purposes
# #     plot_path(env.obstacles, start, goal, path, title="Environment Visualization")

import time
import numpy as np
import matplotlib.pyplot as plt
from cw1_team_2.environment.map import Environment
from cw1_team_2.environment.object_follower import ObjectFollower
from cw1_team_2.utils.config import CONFIG

def update_plot(ax, path):
    """更新 Matplotlib 画布"""
    ax.clear()

    # 绘制障碍物
    for obs in env.obstacles:
        ax.add_patch(plt.Rectangle((obs["x_min"], obs["y_min"]),
                                   obs["x_max"] - obs["x_min"],
                                   obs["y_max"] - obs["y_min"],
                                   color="black"))

    # 绘制起点 & 目标点
    ax.scatter(*start, color="green", label="Start", s=100)
    ax.scatter(*goal, color="blue", label="Goal", s=100)

    # 绘制路径
    if len(path) > 1:
        ax.plot([p[0] for p in path], [p[1] for p in path], "r-", linewidth=2)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.legend()
    plt.title("Real-time Obstacle Following Visualization")
    plt.draw()
    plt.pause(0.3)  # 增加 pause 时间，使变化更明显

if __name__ == "__main__":
    env = Environment(CONFIG["map_file"])
    start = (0.0, 0.0)
    goal = (4.0, 4.0)
    path = [start]

    follower = ObjectFollower(env)
    follower.current_pos = np.array(start)

    plt.ion()  # 开启交互模式
    fig, ax = plt.subplots()

    for _ in range(30):  # 假设绕障30步
        new_pos = follower.move_around_obstacle()
        path.append(tuple(new_pos))  # 存入轨迹
        update_plot(ax, path)  # 更新可视化

    path.append(goal)  # 添加目标点
    update_plot(ax, path)
    
    plt.ioff()  # 关闭交互模式
    plt.show()

