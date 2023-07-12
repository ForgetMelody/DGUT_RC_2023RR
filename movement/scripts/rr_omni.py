#!/usr/bin/env python3
import rospy
from math import pi
from geometry_msgs.msg import Twist
from serial_pkgs.msg import Cmd, MotorState
from std_msgs.msg import Int16

sqrt2_2 = 0.707106  # 2分之根号2
sqrt2 = 1.41421  # 根号2
radius2dgree = 57.2957  # 弧度转角度


class movement_control:
    def __init__(self):
        rospy.init_node('rr_omni_base')
        self.is_controlling = False
        self.wheel_radius = 0.07625
        self.base_radius = 0.260
        self.speed_scale = 21  # 减速比(或者erpm与rpm的转换倍率)
        self.max_rpm = 1900  # 最大转速 40000 erpm / 21 = 1904.7619 rpm
        self.current_vel = Twist()
        # self.max_speed = 3.769  # 最大速度(vx+vy+vw) no used

        # 订阅/cmd_vel 和 cmd_pos
        self.vel_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self.vel_callback, queue_size=100)
        self.pos_sub = rospy.Subscriber(
            "/cmd_pos", Twist, self.pos_callback, queue_size=100)
        # 发布/wheel_speed话题以发布轮子转速控制指令
        self.wheel_pub = rospy.Publisher("/serial/pub", Cmd, queue_size=10)
        rospy.Timer(rospy.Duration(0.01), self.control_on_time)
        rospy.spin()

    def control_on_time(self, _):
        if self.is_controlling == True:
            return
        rpm = [0, 0, 0, 0]
        top = Cmd()
        top.cmd = "S"

        # x 前后 y 左右 z旋转
        x = self.current_vel.linear.x
        y = -self.current_vel.linear.y  # 向右为正
        z = -self.current_vel.angular.z * radius2dgree  # 逆时针为正 弧度/秒

        # 记录绝对值最大rpm，用于后期判断是否超出
        max_rpm = 0
        # 系数
        fun = [[1, 1], [-1, 1], [-1, -1], [1, -1]]
        # 转换单位 从 控制指令的 m/s 转换为 rpm
        w_rpm = z * self.base_radius / 3 / self.wheel_radius
        x_rpm = x * 30 / pi / self.wheel_radius
        y_rpm = y * 30 / pi / self.wheel_radius
        for i in range(len(rpm)):
            rpm[i] = fun[i][0] * sqrt2_2 * x_rpm + \
                fun[i][1] * sqrt2_2 * y_rpm + w_rpm
            if abs(rpm[i]) > max_rpm:  # 记录最大值
                max_rpm = abs(rpm[i])
        # 如果超转速 等比缩小
        if max_rpm > self.max_rpm:
            scale_rate = max_rpm / self.max_rpm
            rpm = list(map(lambda x: x*scale_rate, rpm))
        # 由实际转速转换为电机输入转速(减速箱之前 或者 erpm)
        rpm = list(map(lambda x: x*self.speed_scale, rpm))
        # 发布话题
        top.value = rpm
        self.wheel_pub.publish(top)

    # 速度控制 cmd_vel -> motor rpm/erpm
    def vel_callback(self, msg: Twist):
        # # lock
        self.current_vel = msg
        # if self.is_controlling == True:
        #     rospy.logerr("cant pub control msg while controlling")
        #     return
        # else:
        #     self.is_controlling = True
        # 计算电机转速

        # unlock
        # self.is_controlling = False

    # 通过接口（电控）实现的位控
    def pos_callback(self, msg: Twist):
        # lock
        if self.is_controlling == True:
            rospy.logerr("wating for reflact")
            return
        else:
            self.is_controlling = True
        topic = Cmd()
        topic.cmd = "P"
        # 计算电机转速

        if msg.linear.z != 0:
            num = msg.linear.z * self.base_radius / (2*pi*self.wheel_radius)
            if num < 0:
                num -= 0.05
            elif num > 0:
                num += 0.05
            turn_rpm = [
                -(num),
                -(num),
                -(num),
                -(num)
            ]
            topic.value = turn_rpm
            self.wheel_pub.publish(topic)
            # wait for relfact
            # 阻塞等待位置环完成
            # BUG: 有时候会出现发送丢包，进而没有返回，因此设置延时
            # rospy.wait_for_message("/serial/PosControl", Int16)

            rospy.sleep(0.01)
        self.is_controlling = False


if __name__ == '__main__':
    sub_and_pub = movement_control()

