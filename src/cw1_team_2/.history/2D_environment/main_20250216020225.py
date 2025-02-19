import time
from environment import Environment
from robot import Robot
from visualization import plot_environment

if __name__ == "__main__":
    env = Environment()  # 创建环境
    robot = Robot(start_pos=(0.5, 0.5), start_angle=0)  # 初始化机器人

    plt.ion()  # 开启 Matplotlib 交互模式

    for _ in range(100):  # 模拟 100 步
        robot.move_forward()  # 机器人前进
        robot.rotate(0.1)  # 每步旋转 0.1 弧度

        plot_environment(env, robot)  # 实时绘制环境和轨迹
        time.sleep(0.1)  # 控制动画速度

    plt.ioff()  # 关闭交互模式
    plt.show()
