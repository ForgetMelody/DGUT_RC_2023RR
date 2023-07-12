#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from target import target_point, nav_goals
from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import tf
import tf.transformations  # !/usr/bin/env python
from move_base_msgs.msg import MoveBaseActionResult


class Nav:
    def __init__(self) -> None:
        rospy.init_node('send_goal_node')
        #
        self.listener = tf.TransformListener()
        self.odom = None  # 当前位姿
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.nav_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)  # 发布导航请求
        rospy.Subscriber("/nav_point", String, self.pub_goal)
        rospy.Subscriber("/nav_aim", Pose, self.aim_goal)
        # rospy.Subscriber("/Odometry", Odometry, self.odom_callback)  # 订阅当前位置信息
        # rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback) #订阅导航结果
        rospy.sleep(1)
        rospy.spin()

    def odom_callback(self, msg: Odometry):
        # 获取位置坐标(获取到的为 xx 坐标系)
        position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        try:
            # 尝试转换到 base_link坐标系中
            (trans, rot) = self.listener.lookupTransform(
                "base_link", "body", rospy.Time(0))
            position_trans = tf.transformations.quaternion_matrix(position)
            orientation_trans = tf.transformations.quaternion_matrix(
                quaternion)
            t_body2base = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot)
            )
            pose_base = tf.transformations.concatenate_matrices(
                t_body2base,
                [0, 0, 0, 1]
            )
            pose_base = tf.transformations.concatenate_matrices(
                tf.transformations.inverse_matrix(pose_base),
                tf.transformations.concatenate_matrices(
                    position_trans, orientation_trans)
            )
        except:
            rospy.logerr("转换odometry frame到 base_link 坐标系失败")
            return
        self.odom = pose_base

    def aim_goal(self, msg: Pose):
        target = PoseStamped()
        r, p, yaw = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        rospy.loginfo("nav aim yaw:" + str(yaw))
        ps = PoseStamped()
        ps.header.frame_id = "init"
        ps.header.stamp = rospy.Time.now()
        ps.pose = msg
        ps.pose.position.x += 0.195
        self.nav_pub.publish(ps)

    def pub_goal(self, point_name: String):
        nav_point_name = point_name.data
        rospy.loginfo("前往导航点" + point_name.data)
        msg = PoseStamped()
        msg.header.frame_id = "init"
        msg.header.stamp = rospy.Time.now()
        if nav_goals.get(nav_point_name) == None:
            rospy.logerr("不存在导航点 [%s]".format(nav_point_name))
        msg.pose.position.y = nav_goals[nav_point_name][1]
        msg.pose.position.x = nav_goals[nav_point_name][0]+0.195
        msg.pose.position.z = 0
        roll, pitch, yaw = 0, 0, nav_goals[nav_point_name][2]
        orientation = list(tf.transformations.quaternion_from_euler(
            roll, pitch, yaw))
        # print("orientation:", orientation)
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        self.nav_pub.publish(msg)

    # def pub(self):
    #     while not rospy.is_shutdown():
    #         self.goal_pub.publish(self.pose_goal)  # 发布目标点
    #     rospy.loginfo("Sending goal success!")

    # def result_callback(self,data):
    #     rospy.loginfo("result:"+str(data))


if __name__ == '__main__':
    nav = Nav()
    # yaw = math.pi/4  # 角度
    # position_set = [3.0, 4.5, 0]
    # orientation_set = [0, 0, math.sin(yaw/2), math.cos(yaw/2)]

    # goal = Setgoal()
    # goal.set(position=position_set, orientation=orientation_set)
    # goal.pub()

