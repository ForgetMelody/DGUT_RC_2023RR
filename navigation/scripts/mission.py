#!/usr/bin/env python3
# 整体的任务流程
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String, Float32, Float64MultiArray, Int16
from tf.transformations import euler_from_quaternion
from serial_pkgs.msg import Cmd
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from target import target_point, nav_goals

flag_pack2ascii = 1234567.0
target_list = [1, 0, 3, 6, 5, 7]  # 发射目标
pitch_list = [43, 40, 43, 36, 51, 38]  # 发射俯仰
speed_list = [2000, 4300, 2700, 7100, 4900, 7100]  # 发射速度
trigger_set = [  # 作为触发式动作的命令集，用string类型的话题统一触发
    -1,  # reload分环 装填环到发射机构中
    -2,  # shoot发射触发 开合摩擦带
    -3,  # get收放取环臂
    -4,  # aim自动瞄准
    -5   # save 保存
]


class Mission:
    def __init__(self):
        rospy.init_node("mission_node")
        rospy.loginfo("开始任务流程，等待指令")
        self.init_pose = None
        self.pose = Pose()
        # 初始化变量
        self.ser_up = rospy.Publisher("/serial/pub_up", Cmd, queue_size=10)
        self.goal_pub = rospy.Publisher("/nav_point", String, queue_size=12)
        self.cmd_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10)
        self.lock_pub = rospy.Publisher(
            "/lock_vel", Twist, queue_size=10)
        self.init_yaw_pub = rospy.Publisher(
            "/init_target_yaw", Float32, queue_size=10)
        self.control_pub = rospy.Publisher(
            "/control", Float64MultiArray, queue_size=10)
        self.switch_pub = rospy.Publisher(
            "/switch_target", Int16, queue_size=10)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        # 开始
        control_msg = Float64MultiArray()
        # 要按一下手柄才执行一次
        rospy.wait_for_message("/next", String, timeout=None)
        # 导航到上坡点
        self.nav_to_point("step_down")
        # 到达导航点 此时 x=4.4,y = -3.6 偏航为pi/2
        # rospy.wait_for_message("/next", String, timeout=None)
        self.climb()
        # 上坡完成 此时x = 4.4,y = -2.4 偏航 pi/2
        # 导航到取环点
        # rospy.wait_for_message("/next", String, timeout=None)
        # 左侧取环点 x = 2.56 y = 0.79 偏航 -pi
        # 右侧取环点 x = 2.56 y = 0.17 偏航 -pi
        # 取到环时 x = 2.85

        self.get_ring("right")
        # 导航到发射点
        self.nav_to_point("shoot")
        # 装弹
        control_msg.data = [-1]
        self.control_pub.publish(control_msg)
        # pitch_msg = Cmd()
        # pitch_msg.cmd = 'C'
        # pitch_msg.value = [17, 0]
        # self.ser_up.publish(pitch_msg)
        # rospy.sleep(0.8)
        # self.ser_up.publish(reload_msg)
        # rospy.sleep(1)
        rospy.wait_for_message("/next", String, timeout=None)

        self.get_ring("left")
        # 导航到发射点
        self.nav_to_point("shoot")
        # 装弹
        control_msg.data = [-1]
        self.control_pub.publish(control_msg)
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
        #     self.switch_pub.publish(aim_msg)
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
        rospy.loginfo("结束")

    def climb(self):
        # 上坡
        climb_msg = Float32()
        climb_msg.data = 1.5708
        twist = Twist()
        self.init_yaw_pub.publish(climb_msg)
        rospy.sleep(0.2)
        for i in range(30):
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        rospy.wait_for_message("/next", String, timeout=None)
        rospy.loginfo("开始上坡")
        twist = Twist()

        speed_set = np.linspace(0, 2, 30)
        for i in speed_set:
            twist.linear.x = i
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)
        while True:
            if self.pose.position.y > -2.38:  # 直到y值超过-2.4(上完坡)
                break
            self.lock_pub.publish(twist)
            rospy.sleep(0.01)

        rospy.loginfo("上坡完成")

    def get_ring(self, LeftOrRight: str):
        # rospy.wait_for_message("/next", String, timeout=None)
        if LeftOrRight == "left":
            nav_name = "get_l"
        if LeftOrRight == "right":
            nav_name = "get_r"
        self.nav_to_point(nav_name)
        # 取环
        # rospy.wait_for_message("/next", String, timeout=None)
        # 放下取环
        control_msg = Float64MultiArray()
        control_msg.data = [-3]
        self.control_pub.publish(control_msg)
        self.wait_for_finish_motion()
        # 后退
        # 锁偏航
        climb_msg = Float32()
        climb_msg.data = -3.1415926
        self.init_yaw_pub.publish(climb_msg)

        twist = Twist()
        twist.linear.x = -0.7
        # self.cmd_pub.publish(twist)

        while True:
            self.lock_pub.publish(twist)
            if self.pose.position.x > 3.3:
                break
        # 停
        twist.linear.x = 0
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        rospy.sleep(0.3)
        # 前进
        twist.linear.x = 0.6
        self.cmd_pub.publish(twist)
        while True:
            if self.pose.position.x < 3.15:
                break
        # 停
        twist.linear.x = 0
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        self.cmd_pub.publish(twist)
        rospy.sleep(0.3)
        # 收
        self.control_pub.publish(control_msg)
        rospy.sleep(0.3)

    def shoot(self, pitch, speed):
        # 俯仰
        control_msg = Float64MultiArray()
        control_msg.data = [pitch, 0]
        self.control_pub.publish(control_msg)
        # 开合
        control_msg.data = [-2]  # 合
        self.control_pub.publish(control_msg)
        self.wait_for_finish_motion()
        # 加速
        control_msg.data = [pitch, speed]
        self.control_pub.publish(control_msg)
        rospy.sleep(0.2)
        # 停
        control_msg.data = [pitch, 0]
        self.control_pub.publish(control_msg)
        self.wait_for_finish_motion()
        # 开
        control_msg.data = [-2]
        self.control_pub.publish(control_msg)
        self.wait_for_finish_motion()
        # 装弹
        control_msg.data = [-1]  # reload
        self.control_pub.publish(control_msg)
        self.wait_for_finish_motion()

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

    # 判断动作是否执行完成
    def wait_for_finish_motion(self):
        while True:
            msg = rospy.wait_for_message("/motion_num", Int16, timeout=None)
            if msg.data == 0:
                return

    def nav_to_point(self, point_name: str):
        # 导航到坡底下
        rospy.loginfo("正在前往导航点"+str(point_name))
        nav_msg = String()
        nav_msg.data = point_name
        for i in range(6):
            self.goal_pub.publish(nav_msg)
            nav_reslut: MoveBaseActionResult = rospy.wait_for_message(
                "/move_base/result", MoveBaseActionResult, timeout=None)
            if nav_reslut.status.status == 3:
                # 等待停止后检测位置，若误差超过则
                rospy.sleep(0.2)
                x_err = abs(self.pose.position.x -
                            (nav_goals[point_name][1]))
                y_err = abs(self.pose.position.y -
                            (nav_goals[point_name][0]))
                yaw_err = abs(self.yaw - nav_goals[point_name][2])

                isFinish: bool = (x_err < nav_goals[point_name][4]) and (
                    y_err < nav_goals[point_name][3]) and (yaw_err < nav_goals[point_name][5])
                rospy.logwarn("target:"+point_name+" err:"+str(x_err)+str(y_err)+ str(yaw_err))
                if isFinish:
                    rospy.loginfo("成功到导航点"+str(point_name))
                    return


if __name__ == "__main__":
    mis = Mission()
