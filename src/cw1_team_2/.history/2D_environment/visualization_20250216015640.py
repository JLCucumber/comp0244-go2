import matplotlib.pyplot as plt
import numpy as np

def plot_environment(env, robot):
    """绘制环境、障碍物和机器人"""
    fig, ax = plt.subplots()
    
    # 绘制障碍物
    for obs in env.obstacles:
        width = abs(obs["x_max"] - obs["x_min"])
        height = abs(obs["y_max"] - obs["y_min"])
        ax.add_patch(plt.Rectangle((obs["x_min"], obs["y_min"]), width, height, color="black"))

    # 绘制机器人
    ax.scatter(*robot.position, color="blue", s=100, label="Robot", edgecolors="black")

    # 绘制机器人朝向
    heading = robot.position + np.array([np.cos(robot.angle), np.sin(robot.angle)]) * 0.2
    ax.plot([robot.position[0], heading[0]], [robot.position[1], heading[1]], "r-", linewidth=2)

    # 绘制传感器范围
    sensor_circle = plt.Circle(robot.position, robot.sensor_range, color="blue", alpha=0.2)
    ax.add_patch(sensor_circle)

    # 绘制传感器探测方向
    sensor_readings = robot.get_sensor_readings(num_rays=8)
    angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    for i, dist in enumerate(sensor_readings):
        ray_end = robot.position + np.array([np.cos(robot.angle + angles[i]), np.sin(robot.angle + angles[i])]) * dist
        ax.plot([robot.position[0], ray_end[0]], [robot.position[1], ray_end[1]], "g--", alpha=0.5)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.legend()
    plt.title("2D Robot Environment")
    plt.show()
