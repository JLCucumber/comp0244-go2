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
from cw1_team_2.environment.map import Environment
from cw1_team_2.environment.object_follower import ObjectFollower
from cw1_team_2.utils.visualization import plot_path
from cw1_team_2.utils.config import CONFIG
import matplotlib.pyplot as plt

if __name__ == "__main__":
    env = Environment(CONFIG["map_file"])
    start = (0, 0)
    goal = (4, 4)
    path = [start]

    follower = ObjectFollower(env)
    follower.current_pos = np.array(start)

    plt.ion()  # 开启交互模式
    fig, ax = plt.subplots()

    for obs in env.obstacles:
        ax.add_patch(plt.Rectangle((obs["x_min"], obs["y_min"]),
                                   obs["x_max"] - obs["x_min"],
                                   obs["y_max"] - obs["y_min"],
                                   color="black"))

    ax.scatter(*start, color="green", label="Start", s=100)
    ax.scatter(*goal, color="blue", label="Goal", s=100)
    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.legend()
    plt.title("Environment and Object Follower Visualization")

    for _ in range(10):  # 假设绕障10步
        new_pos = follower.move_around_obstacle()
        path.append(tuple(new_pos))
        follower.current_pos = new_pos

        ax.plot([p[0] for p in path], [p[1] for p in path], "r-")
        plt.draw()
        plt.pause(0.5)  # 暂停0.5秒以实现实时效果

    path.append(goal)  # 添加目标点到路径
    ax.plot([p[0] for p in path], [p[1] for p in path], "r-")
    plt.ioff()  # 关闭交互模式
    plt.show()