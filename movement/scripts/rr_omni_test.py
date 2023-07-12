#!/usr/bin/env python3
import rospy
from math import pi
from geometry_msgs.msg import Twist
from serial_pkgs.msg import Cmd, MotorState
from std_msgs.msg import Int16

sqrt2_2 = 0.707106


class movement_control:
    def __init__(self):
        rospy.init_node('rr_omni_base')
        self.is_controlling = False
        self.wheel_radius = 0.075
        self.base_radius = 0.248
        self.speed_scale = 19  # 减速比
        self.max_rpm = 480  # 最大转速
        self.max_speed = 3.769  # 最大速度(vx+vy+vw)

        # 订阅/cmd_vel 和 cmd_pos
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        self.pos_sub = rospy.Subscriber("/cmd_pose", Twist, self.pos_callback)
        # 发布/wheel_speed话题以发布轮子转速控制指令
        self.wheel_pub = rospy.Publisher("/serial/pub", Cmd, queue_size=10)

        rospy.spin()

    def vel_callback(self, msg: Twist):
        # lock
        if self.is_controlling == True:
            rospy.logerr("cant pub control msg while controlling")
            return
        else:
            self.is_controlling == True
        # 计算电机转速
        rpm = [0, 0, 0, 0]
        top = Cmd()
        top.cmd = "S"

        # x 前后 y 左右 z旋转
        x = msg.linear.x
        y = -msg.linear.y
        z = -msg.angular.z*57.2957

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
        # scale to motor rpm
        rpm = list(map(lambda x: x*self.speed_scale, rpm))
        # 发布话题
        top.value = rpm
        self.wheel_pub.publish(top)
        # unlock
        self.is_controlling = False

    def pos_callback(self, msg: Twist):
        # lock
        if self.is_controlling == True:
            rospy.logerr("cant pub control msg while controlling")
            return
        else:
            self.is_controlling == True
        topic = Cmd()
        topic.cmd = "P"
        # 计算电机转速

        if msg.linear.x != 0:
            num = msg.linear.x/pi/self.wheel_radius/2*sqrt2_2
            x_rpm = [19*1*num, -1*19*num, -1*19*num, 1*19*num]
            topic.value = x_rpm
            self.wheel_pub.publish(topic)
            # wait for relfact
            reflact = rospy.wait_for_message("/serial/PosControl", Int16)
            rospy.sleep(0.1)
        if msg.linear.y != 0:
            num = msg.linear.y/pi/self.wheel_radius/2*sqrt2_2
            y_rpm = [1*19*num, 1*19*num, -1*19*num, -1*19*num]
            topic.value = y_rpm
            self.wheel_pub.publish(topic)
            # wait for relfact
            reflact = rospy.wait_for_message("/serial/PosControl", Int16)
            rospy.sleep(0.1)
        if msg.linear.z != 0:
            num = msg.linear.z/360*self.base_radius/self.wheel_radius
            turn_rpm = [19*num, 19*num, 19*num, 19*num]
            topic.value = turn_rpm
            self.wheel_pub.publish(topic)
        self.is_controlling = False


if __name__ == '__main__':
    sub_and_pub = movement_control()
