import pandas as pd
import numpy as np
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

def optimize_acc_threshold(imu_file, contact_file):
    imu_data = pd.read_csv(imu_file)
    contact_data = pd.read_csv(contact_file)
    
    imu_time = imu_data.iloc[:, 0].to_numpy()
    contact_time = contact_data.iloc[:, 0].to_numpy()
    
    acc_x = imu_data.iloc[:, 8].to_numpy()
    acc_y = imu_data.iloc[:, 9].to_numpy()
    acc_z = imu_data.iloc[:, 10].to_numpy()
    acc_magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    
    aligned_acc = np.interp(contact_time, imu_time, acc_magnitude)
    contact_states = contact_data.iloc[:, 1:5].to_numpy()  # 4 条腿的接触状态
    
    thresholds_dict = {}
    foot_names = ["Leg0", "Leg1", "Leg2", "Leg3"]
    
    plt.figure(figsize=(12, 8))
    for leg_idx in range(4):
        leg_contact = contact_states[:, leg_idx].astype(int)
        

        ### Original
        # min_acc, max_acc = acc_magnitude.min(), acc_magnitude.max()
        # thresholds = np.linspace(min_acc, max_acc, 20)
        # best_acc = 0
        # best_threshold = 0
        
        # for threshold in thresholds:
        #     predicted = (aligned_acc > threshold).astype(int)
        #     accuracy = np.mean(predicted == leg_contact)
        #     if accuracy > best_acc:
        #         best_acc = accuracy
        #         best_threshold = threshold
        
        ### Grid Search (Hysteresis FSM version)
        min_acc, max_acc = acc_magnitude.min(), acc_magnitude.max()
        thresholds = np.linspace(max(0, min_acc), max_acc, 20)  # 强制从 0 开始
        best_acc = 0
        best_tau_down, best_tau_up = 0, 0
        best_pred = None

        for i in range(len(thresholds)):
            for j in range(i + 1, len(thresholds)):
                tau_down, tau_up = thresholds[i], thresholds[j]
                predicted = hysteresis_fsm(aligned_acc, tau_down, tau_up)
                acc = np.mean(predicted == leg_contact)
                if acc > best_acc:
                    best_acc = acc
                    best_tau_down, best_tau_up = tau_down, tau_up
                    best_pred = predicted
        
        # print(f"{foot_names[leg_idx]} - Best Accuracy: {best_acc * 100:.2f}%, Best acc_threshold: {best_threshold:.3f}")   # original 
        print(f"{foot_names[leg_idx]} - Best Accuracy: {best_acc * 100:.2f}%, tau_down: {best_tau_down:.3f}, tau_up: {best_tau_up:.3f}")
        
        # thresholds_dict[foot_names[leg_idx]] = best_threshold
        
        plt.subplot(2, 2, leg_idx + 1)
        plt.plot(contact_time, aligned_acc, 'k', label="IMU Acceleration Magnitude")
        plt.plot(contact_time, leg_contact, 'r', label="Ground Truth")
        plt.plot(contact_time, best_pred.astype(int), 'b', label="Predicted")
        plt.axhline(best_tau_down, linestyle="--", color="blue", label="tau_down")
        plt.axhline(best_tau_up, linestyle="--", color="red", label="tau_up")
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration Magnitude (m/s^2)")
        plt.title(f"{foot_names[leg_idx]}: Acceleration vs. Threshold")
        plt.legend()
    
    plt.show()
    plt.tight_layout()
    plt.savefig(f"{output_dir}/{rosbag_name}/imu_thresholds_plot.png")
    print(f"Thresholds plot saved to {output_dir}/{rosbag_name}/imu_thresholds_plot.png")
    
    return thresholds_dict

if __name__ == "__main__":
    output_dir = "/workspace/comp0244-go2/src/cw2_team_2/dataset/outputs"
    rosbag_name = "rosbag2_2025_03_18-23_44_53"  # rosbag2_2025_03_18-23_42_57, rosbag2_2025_03_18-23_32_58

    imu_file = f"{output_dir}/{rosbag_name}/output_imu.csv"
    contact_file = f"{output_dir}/{rosbag_name}/output_contacts.csv"

    thresholds = optimize_acc_threshold(imu_file, contact_file)
    for leg, threshold in thresholds.items():
        print(f"{leg}: Optimized acc_threshold: {threshold:.3f}")