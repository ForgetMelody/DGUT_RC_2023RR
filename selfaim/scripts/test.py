#!/usr/bin/env python3
import rospy
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import struct
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from math import fabs
from filterpy.kalman import KalmanFilter
import time
from scipy.spatial import cKDTree

class custompoint2scan:
    def __init__(self) -> None:
        rospy.init_node("custompoint2scan_node")
        self.position_filter = PositionFilter()
        set_ClusterTolerance=0.02  #设置聚类的距离阈值
        self.pub = rospy.Publisher("/cloud", PointCloud2, queue_size=1)  #发布的点云
        self.pub_split_cloud=rospy.Publisher("/split_cloud",PointCloud2,queue_size=1)
        self.pub_target_cloud=rospy.Publisher("/target_cloud",PointCloud2,queue_size=1)

        self.target_name=['L1','M1','R1','LF2','RF2','A','LB2','RB2']
        self.target_lambda=dict.fromkeys(self.target_name,[])

        self.target=[[2.15,3.04],  #左一型
                     [2.29,0.39],  #中一型
                     [2.31,-2.75],  #右一型
                     [4.17,1.74],  #左前二型
                     [4.18,-0.85],  #右前二型
                     [5.45,0.41],  #a柱
                     [6.74,1.72],  #左后二型
                     [6.72,-0.81]]  #右后二型
        self.limit_range = 0.02  # 匹配点云的限制距离
        self.limit_with=0.01  #一个点云分组中的点之间的限制的曼哈顿距离

        while not rospy.is_shutdown():
            msg=rospy.wait_for_message("/livox/lidar",CustomMsg,timeout=1)  #CustomMsg的msg
            msg_odometry=rospy.wait_for_message("/Odometry", Odometry,timeout=1)  #Odometry的msg

            # start=time.time()
            self.pos_filtered=self.Odometry_callback(msg_odometry)  #得到lidar的相对于camera_init的坐标和偏航
            # end=time.time()
            # print("雷达偏航和坐标所用时间:",end-start)

            # start=time.time()
            self.cloud_msg,self.cloud_points=self.CustomMsg2PointCloud2_callback(msg)  #得到二维点云和带分割的点云
            self.pub.publish(self.cloud_msg)
            # end=time.time()
            # print("压缩点云所用时间:",end-start)

            # start=time.time()
            self.trans_target=self.point_trans()  #转换了坐标之后的target
            # end=time.time()
            # print("target转换时间所用时间:",end-start)


            if len(self.cloud_points)>0:  #发布分割点云
                # start=time.time()
                self.point_group=self.split_cloud(set_ClusterTolerance)
                self.split_cloud_msg=self.ros2PointCloud2(self.point_group)
                self.pub_split_cloud.publish(self.split_cloud_msg)
                # end=time.time()
                # print("分割点云和打上标签所用时间:",end-start)

                start=time.time()
                # 匹配点云
                self.target_point=self.match_cloud()
                end=time.time()
                print("匹配点云所用时间:",end-start)

                self.target_cloud_msg=self.ros2PointCloud2(self.target_point)
                self.pub_target_cloud.publish(self.target_cloud_msg)

    def Odometry_callback(self,msg: Odometry):  #读取Odometry，即雷达信息
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)  #得到偏航

        pos=np.array([x,y,yaw])
        pos_filtered = self.position_filter.update(pos)

        return pos_filtered

    def CustomMsg2PointCloud2_callback(self,msg : CustomMsg):  #CustomMsg转点云的callback
        # 获得CustomMsg中的CustomPoint
        custompoint=msg.points

        # 创建一个 PointCloud2 消息对象
        cloud_msg = PointCloud2()

        # 设定消息头部信息
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = "body"

        # 设置消息中每个点的坐标数据类型为 float32
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        cloud_msg.fields = fields

        # 设置消息中每个点的步长和偏移量
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * 3

        # 填充点云数据
        points_data = []
        cloud_points=[]
        width=0
        for i in custompoint:
            if 0<i.z<0.5:  #筛选中间层的点云
                x = i.x
                y = i.y
                z = 1.0
                points_data.append(struct.pack('fff', x, y, z))

                if (not math.isinf(x)) and (not math.isinf(y)):
                    points_z=atan2(x,y)
                    cloud_points.append([x,y,points_z])
                width+=1
        cloud_msg.data = b''.join(points_data)

        cloud_points=sorted(cloud_points,key= lambda x: x[2])  #处理为根据角度大小排序的有序点云

        # 设置消息中的点数和是否包含无效点数据
        cloud_msg.width = width
        cloud_msg.height = 1
        cloud_msg.is_dense = True

        return cloud_msg,cloud_points
    
    def point_trans(self):  #转换目标点的坐标
        x,y,yaw=self.pos_filtered

        yaw_array=np.array([[np.cos(yaw),-np.sin(yaw)],  #旋转矩阵
                            [np.sin(yaw),np.cos(yaw)]])

        trans_target=[]  #存储转换后的坐标

        for i in self.target:  #计算存储的坐标
            x_old=i[0]-x  #平移
            y_old=i[1]-y

            point_array=np.array([[x_old,y_old]])
            result=np.dot(point_array,yaw_array)  #旋转

            x_new,y_new=result[0][0],result[0][1]
            trans_target.append([x_new,y_new])

        return trans_target
    
    def inside_ok(self,points):  #检验点云组是否合理
        # 找到所有可能成为距离最远的两个点
        n = len(points)
        max_dist = 0.0
        for i in range(n):
            for j in range(i+1, n):
                # 计算两点之间的曼哈顿距离
                dist = float(fabs(points[i][0] - points[j][0]) + fabs(points[i][1] - points[j][1]))
                # 更新最大距离
                max_dist = max(max_dist, dist)
        
        if max_dist <= self.limit_with:
            return True
        else:
            return False
    
    def split_cloud(self,set_ClusterTolerance):  #分割点云的函数
        point_group=[]  #大组
        temp=[]  #小组
        for i in range(len(self.cloud_points)):
            if i == 0:
                temp.append([self.cloud_points[i][0],
                             self.cloud_points[i][1]])
            elif i ==  len(self.cloud_points)-1:
                temp.append([self.cloud_points[i][0],
                             self.cloud_points[i][1]])
                combine_before = False
                for j in point_group:
                    if fabs(j[-1][0]-temp[0][0])+ fabs(
                        j[-1][1]-temp[0][1]) <= set_ClusterTolerance:

                        j.extend(temp)
                        combine_before = True
                        break
                
                if not combine_before:
                    point_group.append(temp)

            elif fabs(self.cloud_points[i][0]-temp[-1][0])+ fabs(
                self.cloud_points[i][1]-temp[-1][1]) <= set_ClusterTolerance:

                temp.append([self.cloud_points[i][0],
                             self.cloud_points[i][1]])
            else:
                combine_before=False
                for j in point_group:
                    if fabs(j[-1][0]-temp[0][0])+ fabs(
                        j[-1][1]-temp[0][1]) <= set_ClusterTolerance:

                        j.extend(temp)
                        combine_before = True
                        break
                
                if not combine_before:
                    point_group.append(temp)
                    self.make_lambda(temp)
                temp=[[self.cloud_points[i][0],self.cloud_points[i][1]]]

        point_group = [x for x in point_group if x != []]

        if fabs(point_group[-1][-1][0]-point_group[0][0][0])+ fabs(
            point_group[-1][-1][1]-point_group[0][0][1]) <= set_ClusterTolerance:

            point_group[0].extend(point_group[-1])
            del point_group[-1]
            self.make_lambda(point_group[0])
        
        point_group = [x for x in point_group if 30<len(x)<50]
        
        print("point_group lenth:",len(point_group))

        return point_group
    
    def make_lambda(self,temps):  #给每一组的点云的分组打上标签
        try:
            lambda_trans=[]
            for temp in temps:
                ttarget=min(self.trans_target,key = lambda x: fabs(x[0]-temp[0])+fabs(x[1]-temp[1]) <= self.limit_range)
                if ttarget not in lambda_trans:
                    lambda_trans.append(ttarget)
                    break
            
            for i in lambda_trans:
                item=self.target_lambda[self.target_name[self.trans_target.index(i)]]
                if temps not in item:
                    item.append(temps)
                    self.target_lambda[self.target_name[self.trans_target.index(i)]]=item
        except:
            print("Make lambda error!\n")
        
    def ros2PointCloud2(self,point_group):  #将分割好的点云坐标封装成点云信息

        cloud_msg = PointCloud2()

        # 设定消息头部信息
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = "body"

        # 设置消息中每个点的坐标数据类型为 float32
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        cloud_msg.fields = fields

        # 设置消息中每个点的步长和偏移量
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * 3

        # 填充点云数据
        points_data = []
        width=0
        for i in point_group:
            for j in i:
                x = j[0]
                y = j[1]
                z = 1.0 
                points_data.append(struct.pack('fff', x, y, z))
                width+=1
        cloud_msg.data = b''.join(points_data)

        # 设置消息中的点数和是否包含无效点数据
        cloud_msg.width = width
        cloud_msg.height = 1
        cloud_msg.is_dense = True

        return cloud_msg
    
    def sift_points(self,n,points):  #精化点云的分组

        sift_point=[]
        while self.inside_ok(sift_point):
            point=min(points,key= lambda x: fabs(self.trans_target[n][0]-x[0])+fabs(self.trans_target[n][1]-x[1]))
            sift_point.append(point)
            points.remove(point)
            if len(sift_point) >=50 or len(points)==0:
                break
        sift_point.pop()

        return sift_point
    
    def match_cloud(self):  #匹配点云
        target_point=[]

        note=0

        for key,value in self.target_lambda.items():
            n=self.target_name.index(key)
            if not value:
                target_point.append(self.trans_target[n])
                note+=1
                continue

            points=[]
            for i in value:
                for j in i:
                    points.append(j)

            new_points = []
            for p in points:
                if p not in new_points:
                    new_points.append(p)
            
            new_points=self.sift_points(n,new_points)
            target_point.append(new_points)
        
        print("选中率:",(len(self.trans_target)-note)/len(self.trans_target)*100,"%")
        return target_point


class PositionFilter:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = [0, 0, 0]  # 初始状态量
        self.kf.H = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])  # 测量矩阵
        self.kf.P *= 1000  # 初始协方差矩阵
        self.kf.R = np.diag([0.1, 0.1, 0.1])  # 测量噪声的协方差矩阵
        self.kf.Q = np.eye(3) * 0.01 # 过程噪声的协方差矩阵

    def update(self, z):
        self.kf.predict()
        self.kf.update(z)
        return self.kf.x

if __name__=="__main__":
    last_update_time = None
    ekf_pos = None
    ekf_cov = None
    vision_pos = None
    custompoint2scan()