#!/usr/bin/env python3
# 导入必要的Python库和ROS消息类型
import rospy
import math
import numpy as np
import os
import subprocess
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String, Int16, Float64MultiArray, Float32
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from selfaim.msg import aim
from sensor_msgs.msg import Joy
from serial_pkgs.msg import Cmd
from math import pi
from target import nav_goals, target_point
import threading
flag_pack2ascii = 1234567.0
name_list = ["1_L", "1_M", "1_R", "2_LF", "2_RF", "ALPHA", "2_LB", "2_RB",]
# 新红  -0.0029 0.001
target_list = [1,    0,    3,    6,    5,    7,    4,    2]  # 发射目标
pitch_list = [47,   32,   39,   33,   47,   33,   39,   32]  # 发射俯仰
speed_list = [2800, 5550, 4300, 6250, 5800, 6250, 4300, 5550]  # 发射速度
# 新蓝  -0.0031 -0.001
# target_list = [1,    0,    3,    6,    5,    7,    4,    2]  # 发射目标
# pitch_list = [47,   33,   39,   35,   46,   35,   39,   33]  # 发射俯仰
# speed_list = [2800, 5450, 4300, 6250, 6000, 6250, 4300, 5450]  # 发射速度
height_list = []  # rr在二层时各个柱子的相对高度
command_set = [  # 作为触发式动作的命令集，用string类型的话题统一触发
    "ps",  # pitch speed
    "reload",  # reload分环 装填环到发射机构中
    "shoot",  # shoot发射触发 开合摩擦带
    "get",  # get收放取环臂
    "aim",  # aim自动瞄准
    "save"   # save 保存发射数据（堆数据用)
]

# 手柄按键映射

# 北通
# 手柄控制集
# axes = {
#     "left_y": 0,
#     "left_x": 1,
#     "LT": 2,
#     "right_y": 3,  # right joy
#     "right_x": 4,
#     "RT": 5,
#     "left_right": 6,  # direction key
#     "up_down": 7
# }
# buttons = {
#     "A": 0,
#     "B": 1,
#     "X": 2,
#     "Y": 3,
#     "LB": 4,
#     "RB": 5,
#     "back": 6,
#     "start": 7,
#     "home": 8,
#     "left_joy": 9,
#     "right_joy": 10
# }
# XBOX
axes = {
    "left_y": 0,  # left joy
    "left_x": 1,
    "LT": 5,
    "right_y": 2,  # right joy
    "right_x": 3,
    "RT": 4,
    "left_right": 6,  # direction key
    "up_down": 7
}
buttons = {
    "A": 0,
    "B": 1,
    "X": 3,
    "Y": 4,
    "LB": 6,
    "RB": 7,
    "back": 10,
    "start": 11,
    "home": 12,
    "left_joy": 13,
    "right_joy": 14
}
# XBOX USB Adapter
# axes = {
#    "left_y": 0,  # left joy
#    "left_x": 1,
#    "LT": 2,
#    "right_y": 3,  # right joy
#    "right_x": 4,
#    "RT": 5,
#    "left_right": 6,  # direction key
#    "up_down": 7
# }
# buttons = {
#    "A": 0,
#    "B": 1,
#    "X": 2,
#    "Y": 3,
#    "LB": 4,
#    "RB": 5,
#    "back": 6,
#    "start": 7,
#    "home": 8,
#    "left_joy": 9,
#    "right_joy": 10
# }


class cmd_controller:
    def __init__(self) -> None:
        # 自瞄数据
        self.target_yaw = []
        self.target_dis = []
        # 状态与限幅相关数据
        self.shoot_pitch = 40  # 默认发射俯仰
        self.pitch_range = [17, 58]  # 俯仰限幅
        self.shoot_speed = 3000  # 默认发射速度
        self.speed_range = [0, 7000]  # 发射限幅
        self.target_index = 4  # 自瞄目标
        self.target_index_last = None
        self.target_range = [0, 7]  # 目标范围
        self.pitch_offset = 0
        self.speed_offset = 0
        self.get_right = True
        self.is_started = False
        self.is_stop = False
        # 定位和手柄数据
        self.pose = None  # 定位
        self.yaw = 0
        self.joy_time = None
        self.joy = None  # 当前手柄数据
        self.last_joy = None  # 上一帧手柄数据
        self.is_lock = False
        # 底盘遥控限幅
        self.zero_count = 0  # 重复发送n次0速后停止发送
        self.zero_num = 3  # 重复发送这么多次0速后停止发送，直到非0速度
        self.max_speed = [3, 3, pi/2]  # 米每秒 弧度每秒
        self.target_speed = [0, 0, 0]  # 目标速度 xyz z轴表自旋
        self.current_speed = [0, 0, 0]  # 当前速度 xyz
        self.is_controlling = False
        self.aim_ing = False
        self.step = [12, 12, 2*pi]  # 加速度 m/s^2
        self.speed_thersold = 999  # 超过速度阈值
        self.update_rate = 0.02  # 速度变换 、 发布间隔 ms
        self.slowdown = [0.5, 0.5, 0.5]  # 超过限幅速度后的降速倍率
        # 初始化ROS节点
        rospy.init_node('joy2cmd')
        rospy.loginfo("joy2cmd node init.")
        self.ser_pub = rospy.Publisher(
            "/serial/pub", Cmd, queue_size=10)  # 底层串口
        self.ser_pub_up = rospy.Publisher(
            "/serial/pub_up", Cmd, queue_size=10)  # 顶层串口
        self.cmd_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)  # 底层
        self.next_pub = rospy.Publisher(
            "/next", String, queue_size=10)  # 流程键(半自动流程)
        self.nav_point = rospy.Publisher(
            "/nav_point", String, queue_size=10)  # 导航点发布
        self.nav_aim = rospy.Publisher(
            "/nav_aim", Pose, queue_size=10)  # 导航实现的自瞄大调整
        self.pos_pub = rospy.Publisher(
            "/cmd_pos", Twist, queue_size=10)
        self.lock_pub = rospy.Publisher(
            '/lock_vel', Twist, queue_size=10)  # 锁轴的vel
        self.switch_target = rospy.Publisher(
            "/switch_target", Int16, queue_size=10)  # 自瞄
        self.init_target_yaw_pub = rospy.Publisher(
            "/init_target_yaw", Float32, queue_size=10)
        self.control_pub = rospy.Publisher(
            "/control", Cmd, queue_size=10)  # 上层
        self.cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10)  # 取消导航
        self.save_pub = rospy.Publisher(
            "/save_log", Float64MultiArray, queue_size=10)  # 保存数据
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/current_state", Float64MultiArray,
                         self.state_callback)
        rospy.Subscriber("/aim_target", aim, self.aim_callback)
        rate = rospy.Rate(1/self.update_rate)

        while not self.is_started:
            rospy.logwarn("wait for started")
            rospy.sleep(0.3)

        mission = threading.Thread(target=self.mission)
        mission.start()

        aim_vel = Twist()
        aim_flag = False
        # esting
        while not rospy.is_shutdown():
            if self.is_started:
                if not ((self.joy is None) or (self.last_joy is None)):
                    if self.joy.buttons[buttons["right_joy"]] == 1:
                        self.is_stop = True
                        cancel_msg = GoalID()
                        cancel_msg.stamp = rospy.Time(0)
                        self.cancel_pub.publish(cancel_msg)  # 取消导航点
                        self.cancel_pub.publish(cancel_msg)  # 取消导航点
                        self.cancel_pub.publish(cancel_msg)  # 取消导航点
                        stop_msg = Twist()
                        self.cmd_pub.publish(stop_msg)  # 停止底盘
                        self.cmd_pub.publish(stop_msg)  # 停止底盘
                        self.cmd_pub.publish(stop_msg)  # 停止底盘
                    elif self.joy.buttons[buttons["right_joy"]] == 0 and self.is_stop == True:
                        self.is_stop = False
                print(self.is_stop)
            # 输入摇杆 -> 移动控制
            # if not self.stop_joy_speed:
                self.update_speed()  # 根据加速度限幅控制速度并发布

            # # 按住X以调整自瞄偏航
            # if not ((self.joy is None) or (self.last_joy is None)):
            #     if self.joy.buttons[buttons["X"]] == 1:
            #         if aim_vel.linear.y < 2.0:
            #             aim_vel.linear.y += 0.05
            #         self.lock_pub.publish(aim_vel)
            #         # 俯仰和速度矫正
            #         self.shoot_speed = self.set_range(
            #             speed_list[self.target_index] + self.speed_offset, self.speed_range)
            #         self.shoot_pitch = self.set_range(
            #             pitch_list[self.target_index]+self.pitch_offset, self.pitch_range)
            #         # 更新目标
            #         msg = Int16()
            #         msg.data = target_list[self.target_index]
            #         self.switch_target.publish(msg)
            #         # 调整自瞄
            #         self.lock_pub.publish(aim_vel)
            # aim_flag = True
            #         rospy.loginfo("正在自动调整偏航")
            #         if self.target_index != self.target_index_last:
            #             self.speed_offset = 0
            #             self.pitch_offset = 0
            #             self.target_index_last = self.target_index  # 更新上一次目标的下标
            # elif self.joy.buttons[buttons["X"]] == 0 and aim_flag == True:
            #     aim_vel.linear.y = 0
            #     self.cmd_pub.publish(aim_vel)
            #     self.cmd_pub.publish(aim_vel)
            #     aim_flag = False
            #         rospy.loginfo("调整结束")

            # 手柄断链超时检测
            # time = rospy.Time.now().to_sec()
            # if time - self.joy_time > 3:
            #     rospy.logerr("超过3秒没有接收到手柄数据,手柄断链")
            # rate.sleep()
            rate.sleep()

    def aim_callback(self, msg: aim):
        self.target_yaw = msg.yaw
        self.target_dis = msg.dis

    def state_callback(self, msg: Float64MultiArray):
        self.shoot_pitch = msg.data[0]
        self.shoot_speed = msg.data[1]
    # 检测按钮按下(仅首次触发)避免重复触发
    # 按下返回true 否则返回false 错误返回None

    def is_axes_pushed(self, key: str, direct: int) -> bool:
        if self.joy is None or self.last_joy is None:
            return None
        if axes.get(key) == None:
            rospy.logerr("[%s]不在手柄axes中".format(key))
        if key in ["left_right", "up_down"]:  # 方向键
            if self.last_joy.axes[axes[key]] != direct and self.joy.axes[axes[key]] == direct:
                return True
            else:
                return False
        else:
            rospy.logwarn("is_axes_pushed 仅处理方向按键")
            return None

    def is_key_pushed(self, key: str) -> bool:
        if self.joy is None or self.last_joy is None:
            return None
        if buttons.get(key) == None:
            rospy.logerr("[%s]不在手柄按键集中".format(key))
            return None
        if self.last_joy.buttons[buttons[key]] == 0 and self.joy.buttons[buttons[key]] == 1:
            return True
        else:
            return False

    # 设置限幅
    def set_range(self, num, rang: list):
        if num > rang[1]:
            return rang[1]
        elif num < rang[0]:
            return rang[0]
        else:
            return num

    # 上层调用接口
    def set_control(self, cmd: list = []):
        if not cmd[0] in command_set:
            rospy.logerr("not exist command" + cmd[0])
            return
        else:
            msg = Cmd()
            msg.cmd = cmd[0]
            msg.value = cmd[1:]
            self.control_pub.publish(msg)

    #
    def auto_aim(self):
        self.aim_ing = True
        self.shoot_speed = self.set_range(
            speed_list[self.target_index] + self.speed_offset, self.speed_range)
        self.shoot_pitch = self.set_range(
            pitch_list[self.target_index] + self.pitch_offset, self.pitch_range)
        # 调整俯仰
        self.set_control(["ps", self.shoot_pitch, 0])
        # 先导航调整
        # self.nav_to_aim(name_list[self.target_index])
        # 后自瞄调整
        # 更新目标
        for i in range(1):
            try:
                yaw_list: aim = rospy.wait_for_message(
                    "/aim_target", aim, timeout=0.1)
                yaw = yaw_list.yaw[target_list[self.target_index]]
            except:
                rospy.logwarn("to long to wait selfaim message")
                yaw = self.target_yaw[target_list[self.target_index]]
            if abs(yaw) < 0.01:
                rospy.loginfo("瞄准结束: err:" + str(yaw))
                self.aim_ing = False
                return
            else:
                aim_vel = Twist()
                aim_vel.linear.z = yaw
                self.pos_pub.publish(aim_vel)
                # try:
                #    rospy.wait_for_message(
                #        "/serial/PosControl", Int16, timeout=0.7)
                # except:
                #    rospy.logwarn("timeout to receive p[]")
                #    pass
                # rospy.sleep(0.1)
        # rospy.sleep(0.1)
        yaw_list: aim = rospy.wait_for_message("/aim_target", aim)
        yaw = self.target_yaw[target_list[self.target_index]]
        rospy.logwarn("瞄准超过阈值:"+str(yaw))
        # aim_vel = Twist()
        # msg = Int16()
        # msg.data = target_list[self.target_index]
        # self.switch_target.publish(msg)
        # # 调整自瞄
        # for i in range(200):
        #     self.lock_pub.publish(aim_vel)
        #     rospy.sleep(0.01)
        #     if self.target_yaw[self.target_index] < 0.025:
        #         self.cmd_pub.publish(aim_vel)
        #         rospy.loginfo("成功瞄准目标 - 最终误差:" +
        #                       str(self.target_yaw[self.target_index]))
        #         break
        # rospy.logwarn("自瞄导航调整结束 - 误差 " +
        #               str(self.target_yaw[self.target_index]) + " 超过阈值")
        # # if self.is_lock:
        #     self.is_lock = False
        # else:
        #     msg = Float32()
        #     msg.data = self.yaw
        #     self.init_target_yaw_pub.publish(msg)
        # self.is_lock = True
        self.aim_ing = False
    # joy话题的回调函数

    def joy_callback(self, data: Joy):
        self.last_joy = self.joy
        self.joy = data
        # update timestamp
        self.joy_time = data.header.stamp.to_sec()  # 用于检测手柄断链
        # msg = Float64MultiArray()
        # 不是实时发布的信息(触发式的按键)
        if not self.is_started:
            reslut = self.is_key_pushed("left_joy")
            if reslut != None and reslut:
                # 启动
                rospy.loginfo("原神！ 启动！")
                # os.system("roslaunch navigation run_robot.launch")
                subprocess.Popen(
                    ["roslaunch", "navigation", "run_robot.launch"])
                # rospy.sleep(5)
                rospy.wait_for_message("/Odometry", Odometry, timeout=None)
                rospy.loginfo("成功启动")
                # 做个动作表示成功启动
                self.set_control(["reload"])

        else:
            # start 键 ->下层主控软复位
            result = self.is_key_pushed("start")
            if result != None and result:
                reset_msg = Cmd()
                reset_msg.cmd = 'C'
                self.ser_pub.publish(reset_msg)

            #  A 键 -> 改变取环臂姿态 0-1触发 收起-放下
            result = self.is_key_pushed("A")
            if result != None and result:
                self.set_control(["get"])
            # B 键 -> 装弹
            result = self.is_key_pushed("B")
            if result != None and result:
                self.set_control(["reload"])
            # Y 键 -> 发射(开合摩擦带)
            result = self.is_key_pushed("Y")
            if result != None and result:

                # 先开合后加速
                # msg.data = [pitch_list[self.current_target],
                #             speed_list[self.current_target]]
                self.set_control(["ps", self.shoot_pitch, self.shoot_speed])

                self.set_control(["shoot"])
                self.wait_for_finish_motion()
                # rospy.sleep(1.0)
                # 后续的事情放入多线程
                rospy.sleep(0.3)
                self.set_control(["shoot"])
                self.set_control(["ps", self.shoot_pitch, 0])
                self.set_control(["reload"])
                # as_thread = threading.Thread(target=self.after_shoot)
                # as_thread.start()

            result = self.is_key_pushed("back")
            if result != None and result:
                self.nav_to_point("shoot")
            # 上下 -> 修改俯仰
            if self.is_axes_pushed("up_down", 1):
                self.pitch_offset += 1
                # self.shoot_pitch = self.set_range(
                #     self.shoot_pitch, self.pitch_range)
                # self.set_control(["ps", self.shoot_pitch, 0])
                self.shoot_pitch = self.set_range(
                    pitch_list[self.target_index]+self.pitch_offset, self.pitch_range)
                rospy.loginfo("当前pitch:" + str(self.shoot_pitch) +
                              " 设置pitch 偏差:"+str(self.pitch_offset))
            if self.is_axes_pushed("up_down", -1):
                self.pitch_offset -= 1
                self.shoot_pitch = self.set_range(
                    pitch_list[self.target_index]+self.pitch_offset, self.pitch_range)
                # self.shoot_pitch = self.set_range(
                #     self.shoot_pitch, self.pitch_range)
                # self.set_control(["ps", self.shoot_pitch, 0])
                rospy.loginfo("当前pitch:" + str(self.shoot_pitch) +
                              " 设置pitch 偏差:"+str(self.pitch_offset))
            # 左右键 切换自瞄目标
            if self.is_axes_pushed("left_right", -1):
                # if self.joy.axes[axes["left_right"]] == -1:
                self.target_index += 1
                if self.target_index == 8:
                    self.target_index = 0
                # self.target_index = self.set_range(
                #     self.target_index, self.target_range)
                self.shoot_pitch = pitch_list[self.target_index]
                self.shoot_speed = speed_list[self.target_index]
                # self.pitch_offset = 0
                # self.speed_offset = 0
                rospy.loginfo("当前目标:"+str(target_list[self.target_index]))
                msg = Int16()
                msg.data = target_list[self.target_index]
                self.switch_target.publish(msg)

            if self.is_axes_pushed("left_right", 1):
                # if self.joy.axes[axes["left_right"]] == 1:
                self.target_index -= 1
                if self.target_index == -1:
                    self.target_index = 7
                # self.target_index = self.set_range(
                #     self.target_index, self.target_range)
                self.shoot_pitch = pitch_list[self.target_index]
                self.shoot_speed = speed_list[self.target_index]
                # self.pitch_offset = 0
                # self.speed_offset = 0
                rospy.loginfo("当前目标:"+str(target_list[self.target_index]))
                msg = Int16()
                msg.data = target_list[self.target_index]
                self.switch_target.publish(msg)
            # x键 自瞄
            if self.is_key_pushed("X"):
                # test = Twist()
                # test.linear.z = 0.2
                # self.pos_pub.publish(test)

                # t
                if not self.aim_ing:
                    rospy.loginfo(
                        "新建自瞄任务，目标："+name_list[target_list[self.target_index]])
                    aim_thread = threading.Thread(target=self.auto_aim)
                    aim_thread.start()
                # 更新发射速度和俯仰

            if self.is_key_pushed("home"):
                msg = String()
                self.next_pub.publish(msg)

            # 左遥感 按下 记录成功数据 ，右遥感按下 记录失败数据
            if self.is_key_pushed("left_joy"):
                msg = Float64MultiArray()
                msg.data = [-5, 1]
                self.save_pub.publish(msg)
            if self.is_key_pushed("right_joy"):
                msg = Float64MultiArray()
                msg.data = [-5, 0]
                self.save_pub.publish(msg)
            # RB LB -> 加减发射速度
            result = self.is_key_pushed("LB")
            if result != None and result:
                self.speed_offset -= 100
                rospy.loginfo("当前发射速度："+str(self.shoot_speed) +
                              " 发射速度偏差" + str(self.speed_offset))
                self.shoot_pitch = self.set_range(
                    pitch_list[self.target_index]+self.pitch_offset, self.pitch_range)

            result = self.is_key_pushed("RB")
            if result != None and result:
                self.speed_offset += 100
                rospy.loginfo("当前发射速度："+str(self.shoot_speed) +
                              " 发射速度偏差" + str(self.speed_offset))
                self.shoot_pitch = self.set_range(
                    pitch_list[self.target_index]+self.pitch_offset, self.pitch_range)

    # 判断动作是否执行完成
    def wait_for_finish_motion(self):
        while True:
            msg = rospy.wait_for_message("/motion_num", Int16, timeout=None)
            if msg.data == 0:
                return

    def nav_to_aim(self, target_name):
        if target_point.get(target_name) is None:
            rospy.logerr("不存在目标:"+target_name)
            return

        for i in range(3):
            curr_x = self.pose.position.x
            curr_y = self.pose.position.y
            tar_x = target_point[target_name][0]
            tar_y = target_point[target_name][1]

            delta_x = tar_x - curr_x
            delta_y = tar_y - curr_y
            yaw = math.atan2(delta_y, delta_x) - math.pi / 2
            ox, oy, oz, ow = quaternion_from_euler(0, 0, yaw)
            msg = Pose()
            msg.position.x = curr_x
            msg.position.y = curr_y
            msg.position.z = 0
            msg.orientation.x = ox
            msg.orientation.y = oy
            msg.orientation.z = oz
            msg.orientation.w = ow
            rospy.loginfo("Nav_aim to point:x"+str(curr_x) +
                          " y" + str(curr_y) + " yaw:"+str(yaw))
            self.nav_aim.publish(msg)
            nav_reslut: MoveBaseActionResult = rospy.wait_for_message(
                "/move_base/result", MoveBaseActionResult, timeout=None)
            if nav_reslut.status.status == 3:
                yaw_err = abs(self.yaw - yaw)
                if yaw_err < 0.025:
                    rospy.loginfo(
                        "成功瞄准目标:"+target_name+" - yaw_err:" + str(yaw_err))
                    return
            else:
                rospy.loginfo("导航不成功:statue:"+str(nav_reslut.status.status))
        rospy.logwarn("瞄准目标"+target_name+"误差超过阈值:" + str(yaw_err))
    # 调用导航接口 跑到指定名称的路径点，并检测位置是否合格

    def nav_to_point(self, point_name: str):
        rospy.loginfo("正在前往导航点"+str(point_name))
        nav_msg = String()
        nav_msg.data = point_name
        for i in range(3):
            self.nav_point.publish(nav_msg)
            nav_reslut: MoveBaseActionResult = rospy.wait_for_message(
                "/move_base/result", MoveBaseActionResult, timeout=None)
            if nav_reslut.status.status == 3:
                # 等待停止后检测位置，若误差超过则
                x_err = abs(self.pose.position.x -
                            (nav_goals[point_name][1]))
                y_err = abs(self.pose.position.y -
                            (nav_goals[point_name][0]))
                yaw_err = abs(self.yaw - nav_goals[point_name][2])
                isFinish: bool = (x_err < nav_goals[point_name][4]) and (
                    y_err < nav_goals[point_name][3]) and (yaw_err < nav_goals[point_name][5])

                if isFinish:
                    rospy.loginfo("成功到导航点"+str(point_name))
                    return

                rospy.logwarn("导航任务:"+point_name+"误差超过阈值: " +
                              str(x_err)+" "+str(y_err)+" " + str(yaw_err))

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        # self.pose.position.x -= self.init_pose.position.x
        # self.pose.position.y -= self.init_pose.position.y
        # self.pose.position.z -= self.init_pose.position.z
        roll, pitch, self.yaw = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])
        if not self.is_started:
            self.is_started = True
            rospy.loginfo("成功启动")
        print("x"+str(self.pose.position.x)+" y" +
              str(self.pose.position.y)+" yaw"+str(self.yaw))

    def update_speed(self):
        # 避免初始化错误
        if self.joy == None or self.last_joy == None:
            return
        # 更新目标速度
        # xy
        self.target_speed[0] = self.max_speed[0]*self.joy.axes[1]
        self.target_speed[1] = self.max_speed[1]*self.joy.axes[0]
        # w
        self.target_speed[2] = self.max_speed[2]*self.joy.axes[2]
        # 避免重复发0
        if self.current_speed == [0, 0, 0] and self.target_speed == [0, 0, 0]:
            self.zero_count += 1
        else:
            self.zero_count = 0

        if self.zero_count > self.zero_num:
            return
        # 根据目标速度更新当前速度
        for i in range(3):
            # 若减速制动 提前反转 和降速
            if self.target_speed[i] == 0 and abs(self.current_speed[i]) > self.speed_thersold:
                # 降速制动
                self.current_speed[i] = self.current_speed[i] * \
                    self.slowdown[i]
            # 根据目标速度进行迭代改变
            if abs(self.target_speed[i] - self.current_speed[i]) < self.step[i]*self.update_rate:
                self.current_speed[i] = self.target_speed[i]
            elif self.target_speed[i] > self.current_speed[i]:
                self.current_speed[i] += self.step[i]*self.update_rate
            elif self.target_speed[i] < self.current_speed[i]:
                self.current_speed[i] -= self.step[i]*self.update_rate
        # 发布
        twist = Twist()
        twist.linear.x = self.current_speed[0]
        twist.linear.y = self.current_speed[1]
        twist.angular.z = self.current_speed[2]
        if self.is_lock:
            twist.angular.z = 0
            self.lock_pub.publish(twist)
        else:
            self.cmd_pub.publish(twist)

    # 自动任务流程
    def mission(self):
        while True:
            rospy.wait_for_message("/next", String, timeout=None)
            if self.pose.position.y < -3.5:
                self.climb()
                # 若上坡失败，此时不会进入导航
                if self.is_stop or self.pose.position.y < -3.5:
                    continue
                self.get_ring("right")
                self.get_right = False
            else:
                if self.get_right:
                    self.get_ring("right")
                    self.get_right = False
                else:
                    self.get_ring("left")
                    self.get_right = True
            self.nav_to_point("shoot")
            # 装弹
            self.set_control(["reload"])
        # 要按一下手柄才执行一次
        # rospy.wait_for_message("/next", String, timeout=None)
        # 导航到上坡点
        # self.nav_to_point("step_down")
        # 到达导航点 此时 x=4.4,y = -3.6 偏航为pi/2
        # rospy.wait_for_message("/next", String, timeout=None)
        # self.climb()
        # 上坡完成 此时x = 4.4,y = -2.4 偏航 pi/2
        # 导航到取环点
        # rospy.wait_for_message("/next", String, timeout=None)
        # 左侧取环点 x = 2.56 y = 0.79 偏航 -pi
        # 右侧取环点 x = 2.56 y = 0.17 偏航 -pi
        # 取到环时 x = 2.85
        # rospy.wait_for_message("/next",String,timeout=None)
        # self.get_ring("right")
        # 导航到发射点

        # pitch_msg = Cmd()
        # pitch_msg.cmd = 'C'
        # pitch_msg.value = [17, 0]
        # self.ser_up.publish(pitch_msg)
        # rospy.sleep(0.8)
        # self.ser_up.publish(reload_msg)
        # rospy.sleep(1)

        # 右侧导航点
        # rospy.wait_for_message("/next", String, timeout=None)

        # self.get_ring("left")
        # 导航到发射点
        # self.nav_to_point("shoot")
        # 装弹
        # self.set_control(["reload"])

        # 导航到各个发射点
        # 发射
        # 射6个柱子
        # for i in range(len(target_list)):
        #     rospy.wait_for_message("/next", String, timeout=None)
        #     index = target_list[i]
        #     pitch = pitch_list[i]
        #     speed = speed_list[i]

        #     # 偏航瞄准
        #     aim_msg = Int16()
        #     aim_msg.data = index
        #     self.switch_target.publish(aim_msg)
        #     self.wait_for_finish_motion()

        #     self.shoot(pitch, speed)

        # rospy.wait_for_message("/next",String,timeout=None)

        # pitch_msg.value = [44, 2600]
        # self.ser_up.publish(pitch_msg)
        # rospy.sleep(1.2)
        # shoot_msg = Cmd()
        # shoot_msg.cmd = 'S'
        # shoot_msg.value = [flag_pack2ascii,
        #                    float(ord(str(' ')))]
        # self.ser_up.publish(shoot_msg)
        # rospy.sleep(1.5)
        # pitch_msg.value = [40, 0]
        # self.ser_up.publish(pitch_msg)
        # rospy.loginfo("结束")

    def climb(self):
        # 上坡
        # 横着走
        climb_msg = Float32()
        climb_msg.data = 0
        twist = Twist()
        self.init_target_yaw_pub.publish(climb_msg)
        rospy.sleep(0.2)
        for i in range(10):
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)
        # rospy.wait_for_message("/next", String, timeout=None)
        rospy.loginfo("开始上坡")
        twist = Twist()

        speed_set = np.linspace(0, 1.8, 50)
        for i in speed_set:
            if self.is_stop:  # 急停时
                return
            if self.pose.position.y > -2.40:  # 直到y值超过-2.4(上完坡)
                twist.linear.y = 0
                self.lock_pub.publish(twist)
                break
            twist.linear.y = i
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)
        while True:
            if self.is_stop:
                return
            if self.pose.position.y > -2.40:  # 直到y值超过-2.4(上完坡)
                twist.linear.y = 0
                self.lock_pub.publish(twist)
                break
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)

        twist.linear.y = 0
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)

        rospy.loginfo("上坡完成")

    def get_ring(self, LeftOrRight: str):
        # rospy.wait_for_message("/next", String, timeout=None)
        # 锁偏航
        climb_msg = Float32()
        climb_msg.data = 3.14159
        self.init_target_yaw_pub.publish(climb_msg)

        twist = Twist()
        if LeftOrRight == "left":
            nav_name = "get_l"
        if LeftOrRight == "right":
            nav_name = "get_r"
        self.nav_to_point(nav_name)
        # 调整偏航
        for i in range(20):
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)
        self.lock_pub.publish(twist)

        # 等待手动调整
        rospy.wait_for_message("/next", String, timeout=None)
        # 加入位置判断 位置不对的时候就结束
        if self.pose.position.y < - 3.5 or self.is_stop:
            return
        # 放下取环
        self.set_control(["get"])
        # 后退
        rospy.sleep(0.4)

        x = 0
        # twist.linear.x = -0.7
        # self.cmd_pub.publish(twist)

        while True:
            if self.is_stop:
                return
            if x > -0.8:
                x -= 0.04
            twist.linear.x = x
            self.lock_pub.publish(twist)
            if self.pose.position.x > 3.3:
                break
            rospy.sleep(0.01)
        # 停
        twist.linear.x = 0
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        rospy.sleep(0.2)
        # 前进
        twist.linear.x = 0.9
        self.cmd_pub.publish(twist)
        while True:
            if self.is_stop:
                break
            if self.pose.position.x < 3.15:
                break
        # 停
        twist.linear.x = 0
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        rospy.sleep(0.2)
        # 收
        self.set_control(["get"])
        # rospy.sleep(0.2)

    def shoot(self, pitch, speed):
        # 俯仰
        self.set_control(["ps", pitch, speed])
        # 开合
        self.set_control(["shoot"])
        self.wait_for_finish_motion()
        # 加速
        self.set_control(["ps", pitch, speed])
        rospy.sleep(0.2)
        # 停
        self.set_control(["ps", pitch, 0])
        self.wait_for_finish_motion()
        # 开
        self.set_control(["shoot"])
        self.wait_for_finish_motion()
        # 装弹
        self.set_control(["reload"])
        self.wait_for_finish_motion()


if __name__ == '__main__':
    ctl = cmd_controller()
