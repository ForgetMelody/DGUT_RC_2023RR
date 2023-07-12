#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import pi
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


# 锁轴移动算法
# 接收 /lock_vel 的Twist后根据目标偏航角添加angular.z的转向速度以应对各种状况
# 例如 ：上坡矫正、原地发射矫正
class climb_algo:
    def __init__(self):
        rospy.init_node("clamb_algo")
        self.Kp = 2.8  # 比例系数 2.53
        self.Ki = 0.0# 积分系数
        self.Kd = 0.28  # 微分系数
        self.err_integral = 0
        self.lass_err = 0
        self.err = 0
        self.dt = 0
        self.last_dt = 0
        self.init_pose = None
        self.pose = None
        self.target_yaw = 0  # 初始姿态，也是目标姿态的偏回
        self.linear = Twist()

        self.cmd_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)  # 转发 locK_vel 处理后发到cmd_vel中
        rospy.Subscriber("/init_target_yaw", Float32,
                         self.init_yaw_callback)  # 设定目标yaw值
        rospy.Subscriber("/lock_vel", Twist, self.cmd_callback)
        # 定位数据（以雷达位置为原点，但是坐标系为base_link，所以初始值不是0
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

        rospy.spin()

    # 设定要锁的轴
    def init_yaw_callback(self, msg: Float32):
        if abs(msg.data) > pi:
            rospy.logerr("[锁轴] 设定值超过正负pi")
            return
        rospy.loginfo("[锁轴]当前锁定yaw值:"+str(msg.data))
        self.target_yaw = msg.data

    # 计算pid
    def odom_callback(self, msg: Odometry):
        if self.init_pose == None:  # 首次获得定位信息时记录下来，作为置零数据
            self.init_pose = msg.pose.pose

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        )
        self.pose = msg.pose.pose
        self.pose.position.x -= self.init_pose.position.x
        self.pose.position.y -= self.init_pose.position.y

        # 计算误差和时间差
        self.lass_err = self.err
        # 取正反转俩个方向的角度误差中较小的一个
        err_1 = self.target_yaw - yaw
        if err_1 > 0:
            err_2 = err_1-pi*2
        elif err_1 < 0:
            err_2 = err_1+pi*2

        if abs(err_1) < abs(err_2):
            self.err = err_1
        else:
            self.err = err_2
        self.last_dt = self.dt
        self.dt = rospy.Time.now().to_sec()

        # 根据误差和时间差计算PID
        self.err_integral += self.err * (self.dt - self.last_dt)
        k = self.err * self.Kp
        i = self.Ki * self.err_integral
        d = self.Kd * (self.err - self.lass_err) / (self.dt - self.last_dt)

        self.angular_z = (k + i + d)
        # 限制幅度
        #if self.angular_z > 1.2:
        #    self.angular_z = 1.2
        #elif self.angular_z < -1.2:
        #    self.angular_z = -1.2

    def cmd_callback(self, msg: Twist):
        twist = Twist()
        twist.linear = msg.linear
        twist.angular.z += self.angular_z  # 添加矫正的旋转速度
        self.cmd_pub.publish(twist)


if __name__ == "__main__":
    algo = climb_algo()
