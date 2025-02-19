import time
from environment import Environment
from robot import Robot
from visualization import plot_environment

if __name__ == "__main__":
    env = Environment()  # 生成 2D 地图
    robot = Robot(env, start_pos=(0.5, 0.5), start_angle=np.pi/4, sensor_range=1.0)  # 创建机器人

    plt.ion()
    for _ in range(20):
        robot.move_forward()  # 让机器人前进
        robot.rotate(np.pi / 12)  # 让机器人旋转
        plot_environment(env, robot)
        time.sleep(0.5)

    plt.ioff()
