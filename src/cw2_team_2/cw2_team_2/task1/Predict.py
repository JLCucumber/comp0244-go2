import torch
import numpy as np
import pandas as pd
import torch.nn as nn
import torch.optim as optim

# 设备选择
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# 定义 LSTM 网络
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.input_size = 10  # IMU 数据有 10 个特征
        self.hidden_size = 84
        self.num_layers = 2
        self.output_size = 4  # 预测 4 个 contact 值

        # 定义 LSTM 和全连接层
        self.lstm = nn.LSTM(self.input_size, self.hidden_size, self.num_layers, batch_first=False)
        self.fc = nn.Linear(self.hidden_size, self.output_size)

        # 损失函数改为 BCEWithLogitsLoss（适用于二分类）
        self.criterion = nn.BCEWithLogitsLoss()
        self.optimizer = optim.AdamW(self.parameters(), lr=0.0001)

    def forward(self, input):
        if input.dim() == 2:
            h0 = torch.zeros(self.num_layers, self.hidden_size).to(input.device)
            c0 = torch.zeros(self.num_layers, self.hidden_size).to(input.device)
        else:
            h0 = torch.zeros(self.num_layers, input.size(1), self.hidden_size).to(input.device)
            c0 = torch.zeros(self.num_layers, input.size(1), self.hidden_size).to(input.device)
        
        out, _ = self.lstm(input, (h0, c0))
        out = self.fc(out)

        # 在推理阶段会使用 torch.sigmoid()，这里不加
        return out

    def train_simple(self, input_data, output_data, num_epochs=10000, batch_size=8):
        self.train()
        
        for epoch in range(num_epochs):
            idx = np.random.randint(0, len(input_data) - 1000, batch_size)
            input_nn = torch.tensor([input_data[i:i+1000] for i in idx], dtype=torch.float32).to(device)
            output_nn = torch.tensor([output_data[i:i+1000] for i in idx], dtype=torch.float32).to(device)

            self.optimizer.zero_grad()
            output_pred = self.forward(input_nn)

            # 计算损失（BCE 适用于 0/1 数据）
            loss = self.criterion(output_pred, output_nn)
            loss.backward()
            self.optimizer.step()

            if epoch % 100 == 0:
                print(f"Epoch {epoch}, Loss: {loss.item()}")

    def predict(self, input):
        self.eval()
        with torch.no_grad():
            input_tensor = torch.tensor(input, dtype=torch.float32).to(device).unsqueeze(1)
            logits = self.forward(input_tensor)  # LSTM 输出 logits

            # 使用 sigmoid 转换为概率，并二值化
            probs = torch.sigmoid(logits)
            binary_output = (probs > 0.5).float()  # 阈值 0.5，转换为 0/1

            return binary_output.cpu().numpy()  # 转换为 numpy

# 预处理数据
def prepare_data(imu_file):
    imu_df = pd.read_csv(imu_file)
    imu_df['timestamp'] = pd.to_numeric(imu_df['timestamp'], errors='coerce')
    imu_df = imu_df.sort_values('timestamp')

    imu_data = imu_df[['orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                        'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                        'linear_acc_x', 'linear_acc_y', 'linear_acc_z']].values

    return imu_df['timestamp'].values, imu_data

# 加载模型
def load_model(model_path):
    net = Net().to(device)
    net.load_state_dict(torch.load(model_path, map_location=device))
    net.eval()
    print("模型加载成功！")
    return net

# 进行预测
def predict(net, imu_data):
    if len(imu_data) < 1000:
        raise ValueError("数据长度不足 1000，无法进行预测！")

    test_input = imu_data[-1000:]
    predictions = net.predict(test_input)  # 使用新的 predict 方法，返回 0/1 结果

    return predictions

# 保存预测结果
def save_predictions(timestamps, predictions, output_file="estimated_contacts.csv"):
    predictions = predictions[:, 0, :]  # 去掉 batch 维度
    predictions_df = pd.DataFrame(predictions, columns=['contact_1', 'contact_2', 'contact_3', 'contact_4'])
    predictions_df.insert(0, 'timestamp', timestamps[-len(predictions):])

    predictions_df.to_csv(output_file, index=False)
    print(f"预测结果已保存至 {output_file}")

if __name__ == '__main__':
    imu_file = '/workspace/ROSBag1/output_imu.csv'
    model_path = 'lstm_model.pth'

    timestamps, input_data = prepare_data(imu_file)
    net = load_model(model_path)
    predictions = predict(net, input_data)
    save_predictions(timestamps, predictions)
