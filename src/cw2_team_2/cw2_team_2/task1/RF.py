import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report

# 1. Read data
imu_data = pd.read_csv('/workspace/comp0244-go2/datasets/Training/output_imu.csv')
effort_data = pd.read_csv('/workspace/comp0244-go2/datasets/Training/output_effort.csv')
contacts_data = pd.read_csv('/workspace/comp0244-go2/datasets/Training/output_contacts.csv')

# 2. Merge aligned feature data (output_imu and output_effort)
features = pd.merge(imu_data, effort_data, on='timestamp', how='inner')

# 3. Align output_contacts timestamps (using nearest neighbor matching)
def nearest_neighbor_align(target_ts, source_ts, source_data, return_df=False):
    aligned_data = []
    for ts in target_ts:
        idx = np.argmin(np.abs(source_ts - ts))
        aligned_data.append(source_data.iloc[idx] if hasattr(source_data, 'iloc') else source_data[idx])
    if return_df:
        return pd.DataFrame(aligned_data)
    return np.array(aligned_data)

# Extract timestamps and contact states
features_ts = features['timestamp'].values
contacts_ts = contacts_data['timestamp'].values

# Align each contact
aligned_contacts = {}
for contact in ['contact_1', 'contact_2', 'contact_3', 'contact_4']:
    aligned_contacts[contact] = nearest_neighbor_align(features_ts, contacts_ts, contacts_data[contact])

# Construct aligned labels DataFrame
labels = pd.DataFrame(aligned_contacts)
labels['timestamp'] = features['timestamp']

# 4. Extract features and labels
X = features.drop(columns=['timestamp'])  # Feature matrix
y = labels[['contact_1', 'contact_2', 'contact_3', 'contact_4']]  # Labels

# 5. Data preprocessing
X = X.fillna(0)  # Fill missing values
y = y.fillna(0).astype(int)  # Ensure labels are integers

# 6. Train random forest models
rf_models = {}
for contact in ['contact_1', 'contact_2', 'contact_3', 'contact_4']:
    rf = RandomForestClassifier(n_estimators=100, random_state=42)
    rf.fit(X, y[contact])  # Train on full data
    rf_models[contact] = rf

# 7. Predict on new ROS bag (test data)
new_imu_data = pd.read_csv('/workspace/comp0244-go2/datasets/Testing/output_imu.csv')
new_effort_data = pd.read_csv('/workspace/comp0244-go2/datasets/Testing/output_effort.csv')    
new_contacts_data = pd.read_csv('/workspace/comp0244-go2/datasets/Testing/output_contacts.csv')

# Merge new data features
new_features = pd.merge(new_imu_data, new_effort_data, on='timestamp', how='inner')

# Extract timestamps
new_features_ts = new_features['timestamp'].values
new_contacts_ts = new_contacts_data['timestamp'].values

# Align new_features to new_contacts_data timestamps
aligned_features = nearest_neighbor_align(
    new_contacts_ts, new_features_ts, new_features.drop(columns=['timestamp']), return_df=True
)
aligned_features = aligned_features.fillna(0)  # Fill missing values
new_X = aligned_features  # This is now aligned to new_contacts_data timestamps

# 8. Predict (using features aligned to new_contacts_data timestamps)
predictions = {}
for contact, model in rf_models.items():
    predictions[contact] = model.predict(new_X)

# # 9. Load ground truth data and align it to new_contacts_data timestamps
# ground_truth_data = pd.read_csv('/workspace/comp0244-go2/datasets/Testing/output_contacts.csv')  # 读取 ground truth 数据
# ground_truth_ts = ground_truth_data['timestamp'].values

# # Align ground truth to new_contacts_data timestamps
# aligned_ground_truth = {}
# for contact in ['contact_1', 'contact_2', 'contact_3', 'contact_4']:
#     aligned_ground_truth[contact] = nearest_neighbor_align(
#         new_contacts_ts, ground_truth_ts, ground_truth_data[contact]
#     )

# # Construct aligned ground truth DataFrame
# ground_truth_labels = pd.DataFrame(aligned_ground_truth)
# ground_truth_labels = ground_truth_labels.astype(int)  # Ensure labels are integers

# # 10. Compare predictions with ground truth
# for contact in ['contact_1', 'contact_2', 'contact_3', 'contact_4']:
#     true_labels = ground_truth_labels[contact]
#     pred_labels = predictions[contact]
#     print(f"\nResults for {contact}:")
#     print(f"Accuracy: {accuracy_score(true_labels, pred_labels)}")
#     print("Classification Report:")
#     print(classification_report(true_labels, pred_labels))

# 11. Save prediction results
pred_df = pd.DataFrame(predictions)
pred_df.insert(0, 'timestamp', new_contacts_data['timestamp'])  # Use new_contacts_data timestamps
pred_df.to_csv('/workspace/comp0244-go2/src/cw2_team_2/cw2_team_2/task1/predicted_contacts.csv', index=False)