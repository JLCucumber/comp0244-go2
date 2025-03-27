import csv
import numpy as np

def read_contacts_with_timestamps(file_path):
    """读取带有时间戳的接触数据"""
    timestamps = []
    contacts = []
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # 跳过标题行
        for row in reader:
            timestamps.append(float(row[0]))  # 假设时间戳在第一列
            contacts.append([int(x) for x in row[1:]])
    return np.array(timestamps), np.array(contacts)

def interpolate_contacts(est_timestamps, estimated_contacts, gt_timestamps):
    """对齐时间戳并插值估计的接触数据"""
    aligned_est_contacts = []
    for gt_time in gt_timestamps:
        est_contact = np.array([np.interp(gt_time, est_timestamps, estimated_contacts[:, leg_idx]) for leg_idx in range(estimated_contacts.shape[1])])
        aligned_est_contacts.append(est_contact)
    return np.round(aligned_est_contacts).astype(int)

def compute_accuracy(estimated_path, ground_truth_path):
    """比较 estimated_contacts.csv 和 output_contact.csv，按时间戳对齐并计算准确率"""
    # 读取估计值和 ground truth
    est_timestamps, estimated_contacts = read_contacts_with_timestamps(estimated_path)
    gt_timestamps, ground_truth_contacts = read_contacts_with_timestamps(ground_truth_path)

    # 对齐时间戳
    aligned_est_contacts = interpolate_contacts(est_timestamps, estimated_contacts, gt_timestamps)

    # 确保两文件行数相同
    if len(aligned_est_contacts) != len(ground_truth_contacts):
        print(f"Error: Number of rows mismatch. Estimated: {len(aligned_est_contacts)}, Ground Truth: {len(ground_truth_contacts)}")
        return

    # 统计每条腿的正确预测
    correct_counts = [0] * 4  # 每条腿的正确预测次数
    total_counts = [0] * 4    # 每条腿的总次数
    total_correct = 0         # 总体正确预测次数
    total_samples = len(aligned_est_contacts)  # 总体样本数

    for est_contacts, gt_contacts in zip(aligned_est_contacts, ground_truth_contacts):
        for leg_idx in range(4):
            total_counts[leg_idx] += 1
            if est_contacts[leg_idx] == gt_contacts[leg_idx]:
                correct_counts[leg_idx] += 1
        # 总体准确率：所有腿在该时间步都正确
        if np.array_equal(est_contacts, gt_contacts):
            total_correct += 1

    # 打印每条腿的准确率
    for leg_idx in range(4):
        accuracy = correct_counts[leg_idx] / total_counts[leg_idx] if total_counts[leg_idx] > 0 else 0
        print(f"Leg {leg_idx} Accuracy: {accuracy:.2%}")
    
    # 打印总体准确率
    overall_accuracy = total_correct / total_samples if total_samples > 0 else 0
    print(f"Overall Accuracy (all legs correct per row): {overall_accuracy:.2%}")

def main():
    # 文件路径（根据实际路径调整）
    estimated_path = '/workspace/estimated_contacts.csv'
    ground_truth_path = '/workspace/ROSBag1/output_contacts.csv'
    
    compute_accuracy(estimated_path, ground_truth_path)

if __name__ == '__main__':
    main()