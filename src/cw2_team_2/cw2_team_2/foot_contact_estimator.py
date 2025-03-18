import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState
from std_msgs.msg import Bool
import csv
import numpy as np

class FootContactEstimator(Node):
    def __init__(self):
        super().__init__('foot_contact_estimator')
        self.low_state_sub = self.create_subscription(
            LowState, '/low_state', self.low_state_callback, 10
        )
        self.contact_pubs = [
            self.create_publisher(Bool, f'/foot_contact/leg_{i}', 10) for i in range(4)
        ]
        self.timer = self.create_timer(0.02, self.process_data)  # 50 Hz
        self.latest_low_state = None
        # 滞后阈值（通过 optimize_thresholds.py 优化）
        self.tau_down = [0.0, 0.0, 0.0, 0.0]  # 每条腿的 tau_down
        self.tau_up = [4.6, 5.0, 4.7, 4.8]    # 每条腿的 tau_up
        # 加速度阈值（可选）
        # self.acc_threshold = 9.3  # 替换为实际值
        # 状态机状态
        self.contact_states = [0] * 4
        # 数据平滑
        self.torque_history = [[] for _ in range(4)]
        self.acc_history = [[] for _ in range(4)]
        self.window_size = 5
        # 记录结果
        self.csv_file = open('estimated_contacts.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'leg0', 'leg1', 'leg2', 'leg3'])

    def low_state_callback(self, msg):
        self.latest_low_state = msg

    def hysteresis_fsm(self, torque, acc_magnitude, leg_idx):
        if self.contact_states[leg_idx] == 0:
            if torque > self.tau_up[leg_idx]:
                self.contact_states[leg_idx] = 1
        elif self.contact_states[leg_idx] == 1:
            if torque < self.tau_down[leg_idx]:
                self.contact_states[leg_idx] = 0
        return self.contact_states[leg_idx]

    def process_data(self):
        if self.latest_low_state is None:
            self.get_logger().warn("No LowState message received yet.")
            return
        timestamp = self.get_clock().now().nanoseconds / 1e9  # 转换为秒
        contacts = []
        for leg_idx in range(4):
            # 提取扭矩（膝关节，假设每条腿 3 个电机，索引为 2, 5, 8, 11）
            torque = self.latest_low_state.motor_state[leg_idx * 3 + 2].tau_est
            # 提取加速度
            acc_x = self.latest_low_state.imu_state.accelerometer[0]
            acc_y = self.latest_low_state.imu_state.accelerometer[1]
            acc_z = self.latest_low_state.imu_state.accelerometer[2]
            acc_magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
            # 平滑数据
            self.torque_history[leg_idx].append(torque)
            self.acc_history[leg_idx].append(acc_magnitude)
            if len(self.torque_history[leg_idx]) > self.window_size:
                self.torque_history[leg_idx].pop(0)
                self.acc_history[leg_idx].pop(0)
            smoothed_torque = sum(self.torque_history[leg_idx]) / len(self.torque_history[leg_idx])
            smoothed_acc = sum(self.acc_history[leg_idx]) / len(self.acc_history[leg_idx])
            # 应用滞后法
            predicted = self.hysteresis_fsm(smoothed_torque, smoothed_acc, leg_idx)
            contact_msg = Bool()
            contact_msg.data = bool(predicted)
            contacts.append(contact_msg.data)
            self.contact_pubs[leg_idx].publish(contact_msg)
        self.csv_writer.writerow([timestamp] + contacts)

    def __del__(self):
        self.csv_file.close()

def main():
    rclpy.init()
    node = FootContactEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()