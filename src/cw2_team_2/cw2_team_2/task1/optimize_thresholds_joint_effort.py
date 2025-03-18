import pandas as pd
import numpy as np
import matplotlib
# matplotlib.use('Agg')  # 使用无图形后端
import matplotlib.pyplot as plt

def hysteresis_fsm(torque_data, tau_down, tau_up):
    """滞后状态机实现"""
    predicted_states = []
    current_state = 0  # 初始为 flight
    for torque in torque_data:
        if current_state == 0 and torque > tau_up:
            current_state = 1
        elif current_state == 1 and torque < tau_down:
            current_state = 0
        predicted_states.append(current_state)
    return np.array(predicted_states)

def optimize_thresholds(output_dir, rosbag_name, torque_file, contact_file):
    # 加载数据
    torque_data = pd.read_csv(torque_file)
    contact_data = pd.read_csv(contact_file)
    
    # 提取时间戳
    torque_time = torque_data.iloc[:, 0].to_numpy()
    contact_time = contact_data.iloc[:, 0].to_numpy()
    
    # 假设每条腿的膝关节扭矩在列 [3, 6, 9, 12]，接触状态在列 [1, 2, 3, 4]
    torque_cols = [3, 6, 9, 12]  # 膝关节
    contact_cols = [1, 2, 3, 4]    # torque_file = "/workspace/output_effort.csv"
    # contact_file = "/workspace/output_contacts.csv"
    foot_names = ["Leg0", "Leg1", "Leg2", "Leg3"]

    # 为每条腿优化阈值
    plt.figure(figsize=(12, 8))
    for leg_idx in range(4):
        
        # 对齐时间戳
        foot_torque = np.interp(contact_time, torque_time, torque_data.iloc[:, torque_cols[leg_idx]])
        foot_contact = np.array(contact_data.iloc[:, contact_cols[leg_idx]].astype(int))
        
        # 网格搜索最佳阈值
        min_t, max_t = foot_torque.min(), foot_torque.max()
        thresholds = np.linspace(max(0, min_t), max_t, 20)  # 强制从 0 开始
        best_acc = 0
        best_tau_down, best_tau_up = 0, 0
        best_pred = None
        for i in range(len(thresholds)):
            for j in range(i + 1, len(thresholds)):
                tau_down, tau_up = thresholds[i], thresholds[j]
                predicted = hysteresis_fsm(foot_torque, tau_down, tau_up)
                acc = np.mean(predicted == foot_contact)
                if acc > best_acc:
                    best_acc = acc
                    best_tau_down, best_tau_up = tau_down, tau_up
                    best_pred = predicted

        # 打印结果
        print(f"{foot_names[leg_idx]} - Best Accuracy: {best_acc * 100:.2f}%, tau_down: {best_tau_down:.3f}, tau_up: {best_tau_up:.3f}")

        # 可视化
        plt.subplot(2, 2, leg_idx + 1)
        plt.plot(contact_time, foot_torque, 'k', label="Joint Torque")
        plt.plot(contact_time, foot_contact, 'r', label="Ground Truth")
        plt.plot(contact_time, best_pred, 'b', label="Predicted")
        plt.axhline(best_tau_down, linestyle="--", color="blue", label="tau_down")
        plt.axhline(best_tau_up, linestyle="--", color="red", label="tau_up")
        plt.xlabel("Time (s)")
        plt.ylabel("Torque")
        plt.title(f"{foot_names[leg_idx]}: Torque vs. Thresholds")
        plt.legend()

    plt.tight_layout()
    plt.show()

    plt.savefig(f"{output_dir}/{rosbag_name}/joint_effort_thresholds_plot.png")
    print(f"Thresholds plot saved to {output_dir}/{rosbag_name}/joint_effort_thresholds_plot.png")

    return best_tau_down, best_tau_up

if __name__ == "__main__":
    # torque_file = "/workspace/output_effort.csv"
    # contact_file = "/workspace/output_contacts.csv"
    output_dir = "/workspace/comp0244-go2/src/cw2_team_2/dataset/outputs"
    rosbag_name = "rosbag2_2025_03_18-23_42_57"  # rosbag2_2025_03_18-23_42_57, rosbag2_2025_03_18-23_32_58, rosbag2_2025_03_18-23_44_53

    torque_file = f"{output_dir}/{rosbag_name}/output_effort.csv"
    contact_file = f"{output_dir}/{rosbag_name}/output_contacts.csv"
    
    best_tau_down, best_tau_up = optimize_thresholds(output_dir, rosbag_name, torque_file, contact_file)