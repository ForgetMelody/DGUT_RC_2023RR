#!/usr/bin/env python3
from math import pi
target_point = {
    "1_L": [2.275, 3.7],  # 左一型
    "1_M": [2.275, 0.5],  # 中一型
    "1_R": [2.275, -2.7],  # 右一型
    "2_LF": [4.175, 1.840],  # 左前二型
    "2_RF": [4.175, -0.8],  # 右前二型
    "alpha": [5.475, 0.5],  # a柱
    "2_LB": [6.775, 1.840],  # 左后二型
    "2_RB": [6.775, -0.8]   # 右后二型
}

# target_point = {
#     # y 横 , x 纵 z 柱子高度(算上台阶)
#     # 定位坐标系 地图右下角为原点
#     #     ↑ x
#     # y ← . z
#     # 一型柱
#     "t_1_L": [9.15, 2.85],  # 左中右
#     "t_1_M": [6.0, 2.85],
#     "t_1_R": [2.85, 2.85],
#     # 2 型柱
#     "t_2_LF": [7.875, 4.125],  # 左前
#     "t_2_LB": [7.875, 7.875],  # 左后
#     "t_2_RF": [4.125, 4.125],  # 右前
#     "t_2_RB": [4.125, 7.875],  # 右后
#     # alpha 柱
#     "t_alpha": [6.0, 6.0]
# }

# start_point = [5.45, 1.01]  # 起始坐标 导航和计算距离的时候需要减去这个偏移

# x y yaw坐标 x y yaw 误差容忍度
nav_goals = {
    "start": [0.5, 0.0, 0, 0.02, 0.02, 0.04],  # 起始点
    # "test1": [9.15, 1.2, 0, 0.02, 0.02, 0.04],  # 三个测试路径点,跑到三个一型柱
    # "test2": [6.0, 1.2, 0, 0.02, 0.02, 0.04],
    # "test3": [2.85, 1.2, 0, 0.02, 0.02, 0.04],
    "step_down": [5.02, -3.75, 0, 0.03, 0.03, 0.03],  # 上坡点
    "step_up": [4.97, -2.2, 0, 0.03, 0.03, 0.03],  # 下坡点
    "get_l": [3.084, 0.913, -pi, 0.02, 0.02, 0.03],  # 左侧取环点
    "get_r": [3.084, 0.269, -pi, 0.02, 0.02, 0.03],  # 右侧取环点
    "shoot2": [5.11, 2.262, -0.4197, 0.02, 0.02, 0.03],
    "shoot": [3.1, 0.625, 0, 0.02, 0.02, 0.02]
    # "shoot": [3.27, 0.519, 0, 0.02, 0.02, 0.02],
}
point_set = ["start", "test1", "test2", "test3",
             "step_down", "step_up", "get_l", "get_r", "shoot"]

