# 可视化工具
import matplotlib.pyplot as plt
import numpy as np

def plot_path(obstacles, start, goal, path=[], title="Bug Path"):
    fig, ax = plt.subplots()

    # 绘制障碍物
    for obs in obstacles:
        ax.add_patch(plt.Rectangle((obs["x_min"], obs["y_min"]),
                                   obs["x_max"] - obs["x_min"],
                                   obs["y_max"] - obs["y_min"],
                                   color="black"))

    # 绘制路径
    if len(path) > 1:
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], "r-", label="Path")

    # 绘制起点和终点
    ax.scatter(*start, color="green", label="Start", s=100)
    ax.scatter(*goal, color="blue", label="Goal", s=100)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.legend()
    plt.title(title)
    plt.show()
