import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os


def hysteresis_fsm(torque_data, tau_down, tau_up):
    '''hysteresis FSM:
    - torque_data: torque data (Nx1)
    - tau_down: torque threshold to transition from high to low
    - tau_up: torque threshold to transition from low to high
    Return: Nx1 array of contact states (0: flight, 1: contact)
    '''

    if torque_data is None:
        print("None!")
        return None

    if not isinstance(torque_data, np.ndarray):
        print("Input data is not a numpy array!")
        return None
    
    contact_states = np.zeros_like(torque_data)
    contact_states[0] = 0 if torque_data[0] < (tau_down + tau_up)/2 else 1 
    
    for i in range (1, contact_states.shape[0]):
        if contact_states[i-1] == 0:
            contact_states[i] = 1 if torque_data[i] > tau_up else 0
        else:
            contact_states[i] = 0 if torque_data[i] < tau_down else 1
    
    return contact_states

dir = "/workspace/datasets/csv"
torque_file = os.path.join(dir, "output_effort.csv")
contact_file = os.path.join(dir, "output_contacts.csv")

torque_data = pd.read_csv(torque_file)
contact_data = pd.read_csv(contact_file)

torque_time = torque_data.iloc[:, 0]
contact_time = contact_data.iloc[:, 0]

for col in contact_data.columns[1:]:
    contact_data[col] = contact_data[col].astype(str).str.lower().map({'true': 1, 'false': 0})

# foot_names = ["Foot1", "Foot2", "Foot3", "Foot4"]
foot_torque_idx = 6 # Adjust based on which foot
foot_contact_idx = 2 # Adjust based on which contact

raw_torque = torque_data.iloc[:, foot_torque_idx]
foot_torque = np.interp(contact_time, torque_time, raw_torque)
foot_contact_gt = contact_data.iloc[:, foot_contact_idx]
time = contact_time

print(f"Foot torque shape: {foot_torque.shape}")
print(f"Foot contact shape: {foot_contact_gt.shape}")
print(f"Time shape: {time.shape}")


num_steps = 10 # Number of discrete threshold values
min_torque, max_torque = foot_torque.min(), foot_torque.max()
threshold_candidates = np.linspace(min_torque, max_torque, num_steps)

best_accuracy = 0
best_tau_down, best_tau_up = None, None
best_pred = None

for i in range(num_steps):
    for j in range(i+1, num_steps):
        tau_down = threshold_candidates[i]
        tau_up = threshold_candidates[j]

        predicted = hysteresis_fsm(foot_torque, tau_down, tau_up)
        accuracy = np.mean(predicted == foot_contact_gt)

        if accuracy > best_accuracy:
            best_accuracy = accuracy
            best_tau_down = tau_down
            best_tau_up = tau_up
            best_pred = predicted
        
    # plt.figure(figsize=(10, 6))

    # # plot torque with best thresholds
    # plt.subplot(2, 1, 1)
    # plt.plot(contact_time.to_numpy(), foot_torque, 'k', linewidth=1, label="Torque")
    # plt.plot(contact_time.to_numpy(), best_pred, label="Contact State", alpha=0.6)
    # plt.axhline(best_tau_down, linestyle="--", color="blue", label="τ_down")
    # plt.axhline(best_tau_up, linestyle="--", color="red", label="τ_up")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Torque")
    # plt.title("Torque vs. Hysteresis Thresholds")
    # plt.legend()

    # # Plot ground truth vs predicted contact state
    # plt.subplot(2, 1, 2)
    # plt.plot(contact_time.to_numpy(), foot_contact_gt.to_numpy(), 'g', linewidth=2, label="Ground Truth")
    # plt.plot(contact_time.to_numpy(), best_pred, 'm', linewidth=1.5, label="Prediction")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Contact State (0/1)")
    # plt.title(f"GT vs. Predicted (Accuracy={best_accuracy * 100:.2f}%)")
    # plt.legend()

    # plt.tight_layout()
    # plt.show(block=False)
    # plt.pause(0.001)  # 短暂暂停以更新图形

    # print("Press 'q' to continue to the next iteration...")
    # while True:
    #     key = plt.waitforbuttonpress()
    #     if key and plt.get_current_fig_manager().canvas.manager.key_press_handler_id == 'q':
    #         break

    # plt.close()  # 关闭当前图形以便下一次迭代

print(f"Best Accuracy: {best_accuracy * 100:.2f}%")
print(f"Best τ_down: {best_tau_down:.3f}, Best τ_up: {best_tau_up:.3f}")

plt.figure(figsize=(10, 6))

# plot torque with best thresholds
plt.subplot(2,1,1)
plt.plot(contact_time.to_numpy(), foot_torque, 'k', linewidth=1, label="Torque")
plt.plot(contact_time.to_numpy(), best_pred, label="Contact State", alpha=0.6)
plt.axhline(best_tau_down, linestyle="--", color="blue", label="τ_down")
plt.axhline(best_tau_up, linestyle="--", color="red", label="τ_up")
plt.xlabel("Time (s)")
plt.ylabel("Torque")
plt.title("Torque vs. Hysteresis Thresholds")
plt.legend()

# Plot ground truth vs predicted contact state
plt.subplot(2, 1, 2)
plt.plot(contact_time.to_numpy(), foot_contact_gt.to_numpy(), 'g', linewidth=2, label="Ground Truth")
plt.plot(contact_time.to_numpy(), best_pred, 'm', linewidth=1.5, label="Prediction")
plt.xlabel("Time (s)")
plt.ylabel("Contact State (0/1)")
plt.title(f"GT vs. Predicted (Accuracy={best_accuracy * 100:.2f}%)")
plt.legend()


plt.tight_layout()
plt.show()