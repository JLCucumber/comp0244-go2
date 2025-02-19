import numpy as np

class Robot:
    """带方向的可移动机器人"""
    
    def __init__(self, env, start_pos=(0.5, 0.5), start_angle=0, sensor_range=1.0, step_size=0.1):
        """
        :param env: 环境对象
        :param start_pos: 机器人起始位置
        :param start_angle: 机器人起始朝向（弧度）
        :param sensor_range: 传感器范围
        :param step_size: 机器人移动的步长
        """
        self.env = env
        self.position = np.array(start_pos)
        self.angle = start_angle  # 方向，以弧度表示
        self.sensor_range = sensor_range
        self.step_size = step_size

    def move_forward(self):
        """沿当前方向前进"""
        dx = self.step_size * np.cos(self.angle)
        dy = self.step_size * np.sin(self.angle)
        new_pos = self.position + np.array([dx, dy])
        if not self.env.is_collision(new_pos):
            self.position = new_pos
        return self.position

    def move_backward(self):
        """沿当前方向后退"""
        dx = -self.step_size * np.cos(self.angle)
        dy = -self.step_size * np.sin(self.angle)
        new_pos = self.position + np.array([dx, dy])
        if not self.env.is_collision(new_pos):
            self.position = new_pos
        return self.position

    def rotate(self, delta_angle):
        """旋转机器人"""
        self.angle += delta_angle  # 角度单位为弧度

    def get_local_obstacles(self):
        """获取局部感知到的障碍物"""
        return self.env.get_obstacles_near(self.position, self.sensor_range)

    def get_sensor_readings(self, num_rays=8):
        """获取 360 度方向上的传感器信息"""
        angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)  # 生成等间距的角度
        readings = []

        for theta in angles:
            ray_dir = np.array([np.cos(self.angle + theta), np.sin(self.angle + theta)])
            for step in np.linspace(0, self.sensor_range, 10):
                test_point = self.position + ray_dir * step
                if self.env.is_collision(test_point):
                    readings.append(step)  # 记录到障碍物的距离
                    break
            else:
                readings.append(self.sensor_range)  # 最大范围

        return readings
