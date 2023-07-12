#!/usr/bin/env python3
import os
import rospy
import datetime
import csv
from std_msgs.msg import Float64MultiArray

type_in_one = [0, 1, 2]  # 一型柱下标
type_in_two = [3, 4, 6, 7]  # 二型柱下标
file_names = ["one.csv", "two.csv", "alpha.csv"]
height = [800, 1000, 1700]  # rr在二层时的相对高度


class CsvLogger:
    def __init__(self):
        rospy.init_node('csv_logger')

        # 打开或创建csv文件
        # 如果不存在csv文件，则新建并插入首行
        for name in file_names:
            if not os.path.isfile(name):
                with open(name, mode='w') as file:
                    writer = csv.writer(file)
                    writer.writerow(
                        ['time', 'x', 'y', 'yaw', 'distance', 'high', 'pitch', 'type', 'hit'])

        # # 写入首行标题
        # self.one_writer.writerow(
        #     ['time', 'x', 'y', 'yaw', 'distance', 'high', 'pitch', 'type', 'hit'])
        # self.two_writer.writerow(
        #     ['time', 'x', 'y', 'yaw', 'distance', 'high', 'pitch', 'type', 'hit'])
        # self.alpha_writer.writerow(
        #     ['time', 'x', 'y', 'yaw', 'distance', 'high', 'pitch', 'type', 'hit'])
        self.sub = rospy.Subscriber(
            '/save_log', Float64MultiArray, self.callback)

        rospy.spin()

    def callback(self, data: Float64MultiArray):
        # 传入的data包含如下内容
        # [x坐标,y坐标,yaw偏航,distance目标距离,high目标高度,pitch发射俯仰，speed发射速度,type目标类型,hit是否命中]
        """
        接收/save_log话题并写入对应的csv文件中
        """
        target_type = data.data[7]  # 获取数据类型字段
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        line = data.data
        line.insert(0, now)
        # 写入对应文件的一行数据
        if target_type in type_in_one:
            with open(file_names[0], mode='a') as file:
                writer = csv.writer(file)
                writer.writerow(line)
        elif target_type in type_in_two:
            with open(file_names[1], mode='a') as file:
                writer = csv.writer(file)
                writer.writerow(line)
        else:
            with open(file_names[2], mode='a') as file:
                writer = csv.writer(file)
                writer.writerow(line)


if __name__ == '__main__':
    logger = CsvLogger()
