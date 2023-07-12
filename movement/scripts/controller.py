#!/usr/bin/env python3
import rospy
import os
from time import sleep
from math import pi
from serial_pkgs.msg import Cmd
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Float64MultiArray, Int16, Float32
from selfaim.msg import aim
flag_pack2ascii = 1234567.0  # 特殊协议，当Cmd.value中第一个为这个数，则后续的数字以ASCII转换为字符发送串口
height = [800, 800, 800, 1000, 1000, 1700, 1000, 1000]
# 注重于动作实现与衔接优化 将这部分从决策层和遥控中移出


class controller:
    def __init__(self):
        self.commands = [  # 作为触发式动作的命令集，用string类型的话题统一触发
            -1,  # reload分环 装填环到发射机构中=
            -2,  # shoot发射触发 开合摩擦带
            -3,  # get收放取环臂
            -4,  # aim自动瞄准
            -5   # save 保存
        ]
        self.motion_queue = [
            # 作为动作队列的命令集
        ]
        self.target_yaw = []
        self.target_dis = []
        self.target_index = 0
        self.target_high = []
        # 相关默认参数与限幅
        self.arm_state = 1  # 取环臂姿态 1为收起 0为放下(取环)
        self.shoot_state = 2  # 摩擦带开合状态 1合 2开
        # 状态的记录
        self.current_speed = 0
        self.current_pitch = 58
        # 发射的记录
        self.shoot_pitch = 40  # 默认发射俯仰
        self.pitch_range = [17, 58]  # 俯仰限幅
        self.shoot_speed = 3000  # 默认发射速度
        self.speed_range = [0, 7000]  # 发射限幅
        # 定位相关数据
        self.init_pose = None
        self.pose = None
        self.yaw = None
        # 订阅和发布相关控制话题
        rospy.init_node("controller")
        rospy.Subscriber("/control",
                         Cmd, self.set_control)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/switch_target", Int16, self.aim)
        rospy.Subscriber("/aim_target", aim, self.aim_callback)
        self.init_yaw_pub = rospy.Publisher(
            "/init_target_yaw", Float32, queue_size=10)
        self.state_pub = rospy.Publisher(
            "/current_state", Float64MultiArray, queue_size=10)
        self.lock_pub = rospy.Publisher(
            "/lock_vel", Twist, queue_size=10)  # 发布锁轴移动
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.ser_pub = rospy.Publisher(
            "/serial/pub", Cmd, queue_size=10)  # 下层串口
        self.ser_pub_up = rospy.Publisher(
            "/serial/pub_up", Cmd, queue_size=10)  # 上层串口
        self.motion_num_pub = rospy.Publisher(
            "/motion_num", Int16, queue_size=10)  # 发布当前剩余动作数量
        self.save_pub = rospy.Publisher(
            "/save_log", Float64MultiArray, queue_size=10)  # 保存当前数据的接口

        # 处理动作队列
        while not rospy.is_shutdown():
            # os.system("clear")
            msg = Int16()
            msg.data = len(self.motion_queue)
            self.motion_num_pub.publish(msg)
            # 完成所有等待动作 进入慢速循环状态 节省资源
            if len(self.motion_queue) == 0:
                sleep(0.1)
                continue
            else:
                # 执行动作队列中的第一个
                motion = self.motion_queue.pop(0)
                cmd: str = motion[0]
                if cmd == 'reload':
                    self.reload()
                elif cmd == 'aim':
                    self.aim()
                elif cmd == 'get':
                    self.get()
                elif cmd == 'shoot':
                    self.shoot()
                elif cmd == 'ps':
                    self.pitch_speed(motion[1], motion[2])
                sleep(0.04)

                # 俯仰 发射速度
                # if isinstance(motion, list) and len(motion) == 2:
                #     if motion[0] == -5 and motion[1] in [0, 1]:
                #         self.save(motion[1])  # 记录数据
                #     else:
                #         self.pitch_speed(motion[0], motion[1])  # 调整俯仰与转速
                # elif isinstance(motion[0], str):
                #     # 分环
                #     if motion == "reload":
                #         self.reload()
                #     elif motion == "aim":
                #         self.aim()
                #     elif motion == "get":
                #         self.get()
                #     elif motion == "shoot":
                #         self.shoot()
                #     sleep(0.04)

    # 回调函数，用于添加队列 和 更新数据
    def set_control(self, msg: Cmd):
        if msg.cmd == 'ps':
            if len(msg.value) == 2:
                self.motion_queue.append(["ps", msg.value[0], msg.value[1]])
        elif msg.cmd == 'reload':  # reload
            self.motion_queue.append(["reload"])
        elif msg.cmd == 'shoot':  # shoot
            self.motion_queue.append(["shoot"])
        elif msg.cmd == 'get':  # get
            self.motion_queue.append(["get"])
        elif msg.cmd == 'aim':  # aim
            self.motion_queue.append(["aim"])
        elif msg.cmd == 'save':
            self.motion_queue.append(["save"])
        elif msg.cmd == 'wait':
            self.motion_queue.append(
                ["wait", msg.value[0], msg.value[1], msg.value[2]])
        else:
            rospy.logerr("err cmd:"+msg.cmd)
            # print(msg.cmd)

    def odom_callback(self, msg: Odometry):
        if self.init_pose == None:
            self.init_pose = msg.pose.pose  # 记录初始值 后续置零点用到
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
        # print("x"+str(self.pose.position.x)+" y" +
        #      str(self.pose.position.y)+" yaw"+str(self.yaw))
    # 更新 自瞄目标

    def aim_callback(self, msg: aim):
        self.target_yaw = msg.yaw
        self.target_dis = msg.dis

    # 发射俯仰与偏航调整的执行器
    def pitch_speed(self, pitch, speed):
        if pitch == self.current_pitch and speed == self.current_speed:
            return
        rospy.loginfo("正在更新俯仰射速:"+str(pitch)+" " + str(speed))
        pitch_msg = Cmd()
        pitch_msg.cmd = 'C'
        # 限幅
        if pitch > self.pitch_range[1]:
            pitch = self.pitch_range[1]
        elif pitch < self.pitch_range[0]:
            pitch = self.pitch_range[0]
        if speed > self.speed_range[1]:
            speed = self.speed_range[1]
        if speed < self.speed_range[0]:
            speed = self.speed_range[0]

        pitch_msg.value = [pitch, speed]
        # 根据俯仰和速度偏差动态调整等待时间
        sleep_time_1 = 1 / \
            (self.pitch_range[1]-self.pitch_range[0]) * \
            abs(pitch - self.current_pitch)  # 最大1.5s按比例算
        sleep_time_2 = 0.7 / \
            self.speed_range[1] * abs(speed - self.current_speed)
        sleep_time = max(sleep_time_1, sleep_time_2)
        self.ser_pub_up.publish(pitch_msg)
        self.current_speed = speed
        self.current_pitch = pitch
        sleep(sleep_time)
    # 发射的执行器

    # 仅仅负责触发发射的摩擦带开合 俯仰与电机加减速以及后续的装弹不在这里
    def shoot(self):
        self.shoot_speed = self.current_speed
        self.shoot_pitch = self.current_pitch
        state_msg = Float64MultiArray()
        state_msg.data = [self.current_pitch, self.current_speed]
        if self.shoot_speed != 0:
            self.state_pub.publish(state_msg)
        rospy.loginfo("改变开合：")
        shoot_msg = Cmd()
        shoot_msg.cmd = 'S'

        # 1 合 2开
        if self.shoot_state == 2:
            self.shoot_state = 1
            sleep_time = 0.5
        else:
            self.shoot_state = 2
            sleep_time = 0.1
        shoot_msg.value = [flag_pack2ascii,
                           float(ord(str(self.shoot_state)))]  # 里面为一个空字符串
        self.ser_pub_up.publish(shoot_msg)
        sleep(sleep_time)

    # 装弹 完成摆到20度俯仰
    def reload(self):
        rospy.loginfo("reload")
        reload_msg = Cmd()
        reload_msg.cmd = 'L'
        reload_msg.value = [flag_pack2ascii, float(ord(str(' ')))]  # 里面为一个空字符串
        self.pitch_speed(20, 0)
        self.ser_pub_up.publish(reload_msg)
        sleep(0.5)
        self.pitch_speed(self.shoot_pitch, 0)

    # 取环臂遥控执行器
    def get(self):
        rospy.loginfo("get")
        arm_msg = Cmd()
        arm_msg.cmd = 'T'
        self.arm_state = (self.arm_state + 1) % 2  # 更新状态
        if self.arm_state == 0:
            # 改变俯仰状态
            self.pitch_speed(44.5, 0.0)
        else:
            self.pitch_speed(38.5, 0.0)
        arm_msg.value = [flag_pack2ascii, float(ord(str(self.arm_state)))]

        self.ser_pub_up.publish(arm_msg)
        sleep(1.3)

    def aim(self, msg: Int16):
        if self.target_yaw is [] or self.target_dis is []:
            return
        target_index = msg.data
        target_yaw = self.target_yaw[target_index]
        target_dis = self.target_dis[target_index]
        # 上面拿到的偏航为相对偏航，还需要转换为绝对偏航
        yaw = self.yaw + target_yaw
        # 限幅
        if yaw < -pi:
            yaw += pi*2
        elif yaw > pi:
            yaw -= pi*2
        msg = Float32()
        msg.data = yaw
        self.init_yaw_pub.publish(msg)
        rospy.loginfo("当前目标:"+str(target_index))
        rospy.loginfo("设置偏航:" + str(yaw))
        # 调整偏航
        # err = 999
        # turn_msg = Twist()
        # for i in range(2):
        #     for j in range(150):
        #         err = abs(self.yaw - yaw)
        #         if err < 0.02:
        #             # 调整完成后停止
        #             self.cmd_pub.publish(turn_msg)
        #             sleep(0.5)
        #             err = abs(self.yaw - yaw)
        #             if err < 0.02:
        #                 break
        #         print("瞄准-err:"+str(err))
        #         self.lock_pub.publish(turn_msg)
        #         sleep(0.01)
        # self.cmd_pub.publish(turn_msg)
        # self.cmd_pub.publish(turn_msg)
        # 更新误差
        # sleep(0.1)

        # target_yaw = self.target_yaw[target_index]
        # yaw = self.yaw + target_yaw
        # # 限幅
        # if yaw < -pi:
        #     yaw += pi*2
        # elif yaw > pi:
        #     yaw -= pi*2
        # msg = Float32()
        # msg.data = yaw
        # self.init_yaw_pub.publish(msg)
        # # sleep(0.1)
        # # 细调整
        # while True:

        #     err = abs(self.yaw - yaw)
        #     if err < 0.02:
        #         # 调整完成后停止
        #         self.cmd_pub.publish(turn_msg)
        #         break
        #     print("细瞄准-err:"+str(err))
        #     self.lock_pub.publish(turn_msg)

        #     sleep(0.05)

    def save(self, isHit: int):
        # 发布相应数据到控制接口上
        msg = Float64MultiArray()
        msg.data = [
            self.pose.position.x,  # x
            self.pose.position.y,  # y
            self.yaw,  # yaw
            self.target_dis[self.target_index],
            height[self.target_index],
            self.shoot_pitch,
            self.shoot_speed,
            self.target_index,
            isHit
        ]
        self.save_pub.publish(msg)


if __name__ == "__main__":
    ctl = controller()
