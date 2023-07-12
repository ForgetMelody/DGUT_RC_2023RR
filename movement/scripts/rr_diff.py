#!/usr/bin/env python3
import rospy
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from serial_pkgs.msg import Cmd
from std_msgs.msg import Int16


class movement_control:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rr_diff_base')
        self.is_controlling = False  # cant speed ctl and pos ctl at the same time
        self.max_rpm = 40000
        self.min_rpm = 400
        self.wheel_distance = 0.432  # 轮子间距
        self.wheel_radius = 0.06  # 轮子半径
        self.real_scale = 78
        self.max_linear_speed = self.max_rpm*pi*self.wheel_radius/30/self.real_scale
        self.max_angular_speed = self.max_rpm*12 * \
            self.wheel_radius/self.wheel_distance/self.real_scale
        print("max_linear_speed:", self.max_linear_speed,
              " max_angular_speed:", self.max_angular_speed)
        # Subscribe /cmd_vel and /cmd_pos
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        self.pos_sub = rospy.Subscriber("/cmd_pose", Twist, self.pos_callback)
        # 发布/wheel_speed话题以发布轮子转速控制指令
        self.wheel_pub = rospy.Publisher("/serial/pub", Cmd, queue_size=10)

        rospy.spin()  # 进入消息循环

    def vel_callback(self, msg):
        # lock
        if self.is_controlling == True:
            rospy.logerr("cant pub control msg while controlling")
            return
        else:
            self.is_controlling == True

        # computing the motor value
        x = msg.linear.x  # 获取线速度
        y = msg.angular.z  # 获取角速度
        if abs(x) > self.max_linear_speed:
            if x > 0:
                x = self.max_linear_speed
            else:
                x = -self.max_linear_speed
        if abs(y) > self.max_angular_speed:
            if y > 0:
                y = self.max_angular_speed
            else:
                y = -self.max_angular_speed

        rpm_diff = abs(y*self.wheel_distance/(self.wheel_radius*12))
        rpm_base = abs(x*30/(self.wheel_radius*pi))

        # turn first
        # if speed to large scale speed let the max rpm equals 40000
        if rpm_diff + rpm_base > self.max_rpm:
            # scale
            scale_rate = self.max_rpm/(rpm_diff+rpm_base)
            rospy.logwarn(
                f"speed requirement [{rpm_base+rpm_diff}] upward than max motor speed")
            rpm_diff *= scale_rate
            rpm_base *= scale_rate
        # if speed to small.scale speed let the min rpm equals 400
        # if rpm_base <self.m
        l_rpm = 0
        r_rpm = 0
        if x > 0:
            l_rpm = rpm_base
            r_rpm = rpm_base
        else:
            l_rpm = -rpm_base
            r_rpm = -rpm_base
        if y > 0:
            l_rpm += rpm_diff
            r_rpm -= rpm_diff
        else:
            l_rpm -= rpm_diff
            r_rpm += rpm_diff

        l_rpm *= self.real_scale
        r_rpm *= self.real_scale
        # 发布左右轮子的转速
        top = Cmd()
        top.cmd = "S"
        top.value = [l_rpm, r_rpm]
        self.wheel_pub.publish(top)
        # unlock
        self.is_controlling = False

    def pos_callback(self, msg):
        # lock
        if self.is_controlling == True:
            rospy.logerr("cant pub control msg while controlling")
            return
        else:
            self.is_controlling == True

        x = msg.linear.x
        y = msg.angular.z
        if (x != 0 and y != 0) or (x == 0 and y == 0):
            rospy.logerr("position control err")
            return
        top = Cmd()
        top.cmd = "P"
        if y == 0:
            # forward & backward
            Trange: float = x/2*pi*self.wheel_radius
            top.value = [Trange, Trange]
        elif x == 0:
            # turn left or right
            Trange: float = y*self.wheel_distance/(pi*720)
            if y > 0:
                # right
                top.value = [Trange, -Trange]
            elif y < 0:
                # left
                top.value = [-Trange, Trange]
            # unlock

        # publishing
        self.wheel_pub.publish(top)
        # wait for relfact
        reflact = rospy.wait_for_message(
            "/serial/PosControl", Int16)
        self.is_controlling = False


if __name__ == '__main__':
    sub_and_pub = movement_control()
