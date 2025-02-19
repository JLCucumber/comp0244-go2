import time

from matplotlib import pyplot as plt
from environment import Environment
from robot import Robot
from visualization import plot_environment

if __name__ == "__main__":
    env = Environment()  # 生成 2D 地图
    robot = Robot(env, start_pos=(0.5, 0.5), start_angle=0.75, sensor_range=0.2)  # 创建机器人

    plt.ion()  # 开启 Matplotlib 交互模式

    for _ in range(50):  # 运行 50 步
        # robot.move_forward()  # 机器人前进
        # robot.rotate(0.1)  # 每步旋转 0.1 弧度
        robot.navigate()  # 机器人导航
        plot_environment(env, robot)  # 实时绘制环境和轨迹
        plt.gcf().canvas.mpl_connect('key_press_event', lambda event: exit(0) if event.key == 'q' else None)
        time.sleep(0.1)  # 控制动画速度

    plt.ioff()  # 关闭交互模式
    plt.show()
