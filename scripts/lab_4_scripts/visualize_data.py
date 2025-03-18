import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os


def hysteresisFSM(torque_data, tau_down, tau_up):
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


''' make notes "dirtily" -- write as much as you want here -- here is your playground
 there's no need to worry about format or writing, feel free to note anything down! '''

dir = "/workspace/datasets/csv"
torque_file = os.path.join(dir, "output_effort.csv")
contact_file = os.path.join(dir, "output_contacts.csv")

torque_data = pd.read_csv(torque_file)
contact_data = pd.read_csv(contact_file)

print(f"Loaded torque file : {torque_data.shape[0]} rows, {torque_data.shape[1]} columns")
print(f"Loaded contact file: {contact_data.shape[0]} rows, {contact_data.shape[1]} columns")

torque_time = torque_data.iloc[:, 0].to_numpy()
contact_time = contact_data.iloc[:, 0].to_numpy()  # Clue for Question 1: For this part, it must be 0 or 1

for col in contact_data.columns[1:]:
    contact_data[col] = contact_data[col].astype(str).str.lower().map({'true': 1, 'false': 0})

torque_cols = [3, 6, 9, 12]  # if it is lower_leg joint (yes)
contact_cols = [1, 2, 3, 4]
foot_names = ["Foot1", "Foot2", "Foot3", "Foot4"]

plt.figure(figsize=(12, 8))

for i in range(4):
    torque_col = torque_cols[i]
    contact_col = contact_cols[i]

    # Question 1: 
    # So here we are actually doing:
    # 1. a hard-core way to align the timestamp --- interpolation
    # 2. What we already have: 
    #   - BOOLEAN at each timestap (ground truth)
    #   - torque data at each timestap
    # 3. What we want to gain:
    #   - torque distribution when contact (non-zero torque is expected) 
    #   - torque distribution of non-contact (nearly zero torque is expected)
    #   - what we wanna see is that:
    #       - when foot contact, joint torque should be around a high value
    #       - when no contact, joint torque should be around zero
    # However, the fact is not like that:
    #   - for "foot-flight": mostly around zero, with few outliers
    #   - for "foot-contact": almost evenly distributed (from 0~15 N) 
    #       - suggesting that the timestamp when a contact happens, the joint can be in many states...
    #       - I don't understand that... why...
    #       - reason 1: "almost contact" - you indeed touch the ground at the moment, but you did not push on it
    #       - reason 2: "interpolation"  - you don't really know what the torque is at certain timestamp of contact array
    #       - reason 3: Eureka!!!
    #            - I know that now !!! --- you yourself must firstly think in a walking scenario  (such as trotting)
    #            - First, assume the joint torque is related with contact force --- i.e., no contact force, no joint torque (sounds reasonable but not very well)
    #            - So, case 1: when no contact --- of course no contact force
    #            - case 2: when contact --- imagine you first touch the ground (nearly no contact force), then push on it (large force), then leave it (force reduce to zero)
    #            - case 2 is a case that explains why nearly joint torques distribute so evenly during foot contact = 1. 
    # So, the way how we settle this problem is by using a "hysteresis" method

    torque_resampled = np.interp(contact_time, torque_time, torque_data.iloc[:,torque_col])
    foot_contact = contact_data.iloc[:, contact_col].to_numpy
    
    # # Approach 1: Dummy threshold
    # torque_contact = torque_resampled[foot_contact == 1]
    # torque_flight = torque_resampled[foot_contact == 0]

    # Approach 2: Hysteresis threshold (coooool !!!)
    # Question 2: wait... why we need to use this method??? what's the point of it?
    #      - I mean, doesn't it sound better (enough) to label joint torque based on ONE threshold?
    #      - Because for contact, when joint torque is below some value 
    # 使用滞后方法的原因是为了更准确地反映接触状态的变化。单一阈值可能会导致频繁的状态切换
    # 例如，当脚刚接触地面时，扭矩可能会有短暂的波动，滞后方法可以避免这些短暂波动导致的状态切换。
    tau_down = 0.1      # foot leaving ground (contact -> non-contact: when torque almost reduce to zero)
    tau_up = 2          # foot touch ground (non-contact -> contact: when torque no lower than 2 - (maybe there is torque in air ??? - for sure )    
    contact_states = hysteresisFSM(torque_resampled, tau_down, tau_up)
        
    plt.subplot(2, 2, i + 1)
    plt.plot(contact_time, torque_resampled, label="Torque")
    plt.plot(contact_time, contact_states, label="Contact State", alpha=0.6)
    plt.title(f"{foot_names[i]} - Torque and Contact State")
    plt.xlabel("Time")
    plt.ylabel("Torque / Contact State")
    plt.legend()
    # plt.subplot(2, 2, i + 1)
    # plt.hist(torque_contact, bins=30, alpha=0.6, color="blue", label="Contact", density=True)
    # plt.hist(torque_flight, bins=30, alpha=0.6, color="red", label="Flight",  density=True)

    # plt.title(f"{foot_names[i]} - Torque Distribution")
    # plt.xlabel("Torque")
    # plt.ylabel("Torque / Contact State")
    # plt.legend()

plt.tight_layout()
plt.show()

