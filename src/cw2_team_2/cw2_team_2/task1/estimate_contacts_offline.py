import csv
import numpy as np

class FootContactEstimator:
    def __init__(self):
        # 动态阈值（根据 optimize_thresholds.py 优化）
        self.tau_down = [0.5, 0.5, 0.5, 0.5]  # 对应 joint_3, joint_6, joint_9, joint_12
        self.tau_up = [4.6, 5.0, 4.7, 4.8]    # 对应 joint_3, joint_6, joint_9, joint_12
        # 状态机状态
        self.contact_states = [0] * 4
        # 数据平滑
        self.torque_history = [[] for _ in range(4)]
        self.window_size = 5
        # 记录结果
        self.csv_file = open('estimated_contacts.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'leg0', 'leg1', 'leg2', 'leg3'])

    def hysteresis_fsm(self, torque, leg_idx):
        """仅基于扭矩的滞后状态机"""
        if self.contact_states[leg_idx] == 0:
            if torque > self.tau_up[leg_idx]:
                self.contact_states[leg_idx] = 1
        elif self.contact_states[leg_idx] == 1:
            if torque < self.tau_down[leg_idx]:
                self.contact_states[leg_idx] = 0
        return self.contact_states[leg_idx]

    def process_data(self, timestamp, torques):
        contacts = []
        for leg_idx in range(4):
            torque = torques[leg_idx]
            
            # 平滑数据
            self.torque_history[leg_idx].append(torque)
            if len(self.torque_history[leg_idx]) > self.window_size:
                self.torque_history[leg_idx].pop(0)
            smoothed_torque = sum(self.torque_history[leg_idx]) / len(self.torque_history[leg_idx])
            
            # 应用滞后法
            predicted = self.hysteresis_fsm(smoothed_torque, leg_idx)
            contacts.append(bool(predicted))
        
        self.csv_writer.writerow([timestamp] + contacts)

    def __del__(self):
        self.csv_file.close()

def main():
    estimator = FootContactEstimator()
    with open('output_effort.csv', 'r') as f:
        reader = csv.reader(f)
        next(reader)  # 跳过标题行
        for row in reader:
            timestamp = float(row[0])  # 第一列是时间戳
            # 提取 joint_3, joint_6, joint_9, joint_12 (索引 3, 6, 9, 12)
            torques = [float(row[3]), float(row[6]), float(row[9]), float(row[12])]
            estimator.process_data(timestamp, torques)
    del estimator

if __name__ == '__main__':
    main()