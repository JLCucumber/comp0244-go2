import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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
        
        min_acc, max_acc = acc_magnitude.min(), acc_magnitude.max()
        thresholds = np.linspace(min_acc, max_acc, 20)
        best_acc = 0
        best_threshold = 0
        
        for threshold in thresholds:
            predicted = (aligned_acc > threshold).astype(int)
            accuracy = np.mean(predicted == leg_contact)
            if accuracy > best_acc:
                best_acc = accuracy
                best_threshold = threshold
        
        print(f"{foot_names[leg_idx]} - Best Accuracy: {best_acc * 100:.2f}%, Best acc_threshold: {best_threshold:.3f}")
        thresholds_dict[foot_names[leg_idx]] = best_threshold
        
        plt.subplot(2, 2, leg_idx + 1)
        plt.plot(contact_time, aligned_acc, 'k', label="Acceleration Magnitude")
        plt.axhline(best_threshold, linestyle="--", color="red", label="Best acc_threshold")
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration Magnitude (m/s^2)")
        plt.title(f"{foot_names[leg_idx]}: Acceleration vs. Threshold")
        plt.legend()
    
    plt.tight_layout()
    plt.savefig("acc_threshold_plot.png")
    
    return thresholds_dict

if __name__ == "__main__":
    imu_file = "/workspace/output_imu.csv"
    contact_file = "/workspace/output_contacts.csv"
    thresholds = optimize_acc_threshold(imu_file, contact_file)
    for leg, threshold in thresholds.items():
        print(f"{leg}: Optimized acc_threshold: {threshold:.3f}")