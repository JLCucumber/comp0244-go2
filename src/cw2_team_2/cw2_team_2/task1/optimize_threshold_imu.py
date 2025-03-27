import torch
import numpy as np
import pandas as pd
import torch.nn as nn
import torch.optim as optim

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.modelname = 'lstm'
        self.input_size = 10       # IMU 数据有 10 个特征
        self.hidden_size = 84
        self.num_layers = 2
        self.batch_size = 8
        self.batch_length = 1000   # 每个序列长度为 1000
        self.output_size = 4       # 预测 4 个 contact 值
        self.input_data = []
        self.output_data = []

        # LSTM 层与线性层
        self.lstm = nn.LSTM(input_size=self.input_size,
                            hidden_size=self.hidden_size,
                            num_layers=self.num_layers,
                            batch_first=True).to(device)
        self.fc = nn.Linear(self.hidden_size, self.output_size).to(device)

        self.optimizer = optim.AdamW(self.parameters(), lr=0.0001)
        self.criterion = nn.MSELoss()
        self.output_norm = 1.0  # 如果 contact 是 0/1 则不归一化

    def forward(self, input_seq):
        # input_seq: (batch, seq_len, input_size)
        h0 = torch.zeros(self.num_layers, input_seq.size(0), self.hidden_size).to(device)
        c0 = torch.zeros(self.num_layers, input_seq.size(0), self.hidden_size).to(device)
        out, _ = self.lstm(input_seq, (h0, c0))  # out: (batch, seq_len, hidden_size)
        out = self.fc(out)  # (batch, seq_len, output_size)
        return out

    def load_train_data(self, input_array, output_array, num_samples=5):
        self.input_data = []
        self.output_data = []
        for _ in range(num_samples):
            idx = np.random.randint(0, input_array.shape[0] - self.batch_length)
            input_seq = input_array[idx:idx + self.batch_length]
            output_seq = output_array[idx:idx + self.batch_length]
            self.input_data.append(input_seq)
            self.output_data.append(output_seq)

    def train_simple(self, num_iterations=1000):
        self.train()
        for j in range(num_iterations):
            for batch in range(len(self.input_data) // self.batch_size):
                # 构造一个 batch
                batch_input = self.input_data[batch * self.batch_size:(batch + 1) * self.batch_size]
                batch_output = self.output_data[batch * self.batch_size:(batch + 1) * self.batch_size]

                input_nn = torch.tensor(batch_input, dtype=torch.float32).to(device)
                output_nn = torch.tensor(batch_output, dtype=torch.float32).to(device) / self.output_norm

                output_pred = self.forward(input_nn)

                loss = self.criterion(output_pred, output_nn)
                if torch.isnan(loss):
                    print("Loss is nan")
                    breakpoint()
                else:
                    self.optimizer.zero_grad()
                    loss.backward()
                    self.optimizer.step()

                if j % 100 == 0 and batch == 0:
                    print(f"[Iter {j:4d}] Loss: {loss.item():.6f}")

    def predict(self, input_seq):
        self.eval()
        with torch.no_grad():
            input_tensor = torch.tensor(input_seq, dtype=torch.float32).to(device)
            if input_tensor.dim() == 2:
                input_tensor = input_tensor.unsqueeze(0)  # 增加 batch 维度
            output_pred = self.forward(input_tensor)
            return output_pred.squeeze(0).cpu().numpy() * self.output_norm  # 返回序列


# 数据预处理
def prepare_data(imu_file, contact_file):
    imu_df = pd.read_csv(imu_file)
    contact_df = pd.read_csv(contact_file)

    imu_df['timestamp'] = pd.to_numeric(imu_df['timestamp'], errors='coerce')
    contact_df['timestamp'] = pd.to_numeric(contact_df['timestamp'], errors='coerce')

    imu_df = imu_df.sort_values('timestamp')
    contact_df = contact_df.sort_values('timestamp')

    merged_df = pd.merge_asof(imu_df, contact_df, on='timestamp', direction='nearest', tolerance=1e6)
    merged_df = merged_df.interpolate(method='linear').dropna()

    imu_data = merged_df[['orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                          'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                          'linear_acc_x', 'linear_acc_y', 'linear_acc_z']].values
    contact_data = merged_df[['contact_1', 'contact_2', 'contact_3', 'contact_4']].values

    print(f"对齐后的数据长度: {len(imu_data)}")
    return imu_data, contact_data


if __name__ == '__main__':
    # 修改为你的数据路径
    imu_file = '/workspace/ROSBag1/output_imu.csv'
    contact_file = '/workspace/ROSBag1/output_contacts.csv'

    input_data, output_data = prepare_data(imu_file, contact_file)

    if len(input_data) < 1000:
        raise ValueError("数据长度不足 1000，无法生成训练序列！")

    net = Net()
    print(net)

    num_samples = min(16, (len(input_data) - net.batch_length) // net.batch_length)
    net.load_train_data(input_data, output_data, num_samples=num_samples)
    print(f"加载了 {len(net.input_data)} 个输入序列用于训练")

    net.train_simple(num_iterations=1000)

    test_input = input_data  # 测试输入
    pred = net.predict(test_input)

    print("预测结果：", pred)
    print("预测形状：", pred.shape)

    torch.save(net.state_dict(), 'lstm_model.pth')
    print("模型已保存为 'lstm_model.pth'")
