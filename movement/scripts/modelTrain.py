#!/usr/bin/env python3
import torch
import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
from std_msgs.msg import Float32MultiArray


class ModelTrainer:
    def __init__(self):
        # 创建节点并订阅数据
        rospy.init_node('model_trainer')
        self.sub = rospy.Subscriber('data', Float32MultiArray, self.callback)

        # 创建模型
        self.model = torch.nn.Sequential(
            torch.nn.Linear(1, 10),
            torch.nn.ReLU(),
            torch.nn.Linear(10, 1)
        )
        # 在__init__中创建一个列表用于存储callback中接收到的训练数据

        # 定义损失函数和优化器
        self.criterion = torch.nn.MSELoss()
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=0.001)

        self.training_data = []
        self.model_path = os.path.expanduser(
            '~/catkin_ws/src/movement/model/model.pth')
        if os.path.isfile(self.model_path):
            self.load_model(self.model_path)

        rospy.spin()
        if rospy.is_shutdown():
            self.save_model(self.model_path)

    def callback(self, msg):
        """
        训练模型并返回预测值
        """
        x = torch.tensor(msg.data[0]).unsqueeze(-1)
        y = torch.tensor(msg.data[1]).unsqueeze(-1)
        self.training_data.append((x, y))

        for epoch in range(10):
            for x, y in self.training_data:
                # 前向计算
                y_pred = self.model(x.float())
                # 计算损失并反向传播
                loss = self.criterion(y_pred, y.float())
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

        x = np.linspace(0, 8, 100)
        x = torch.tensor(x).unsqueeze(-1)
        y_pred = self.model(x.float())
        y_pred = y_pred.detach().numpy()
        img = np.zeros((512, 512, 3), np.uint8)
        for i in range(1, len(y_pred)):
            cv2.line(img, (int(x[i-1]*50), int(y_pred[i-1]*50)),
                     (int(x[i]*50), int(y_pred[i]*50)), (255, 0, 0), 2)
        cv2.imshow('Prediction', img)
        cv2.waitKey(1)

    def save_model(self, path):
        """
        保存模型到指定路径
        """
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save(self.model.state_dict(), path)

    def load_model(self, path):
        """
        从指定路径加载模型
        """
        self.model.load_state_dict(torch.load(path))


if __name__ == '__main__':
    trainer = ModelTrainer()
