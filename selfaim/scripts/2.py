#!/usr/bin/env python3
import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import struct
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from math import fabs
from filterpy.kalman import KalmanFilter
from scipy.spatial import cKDTree
from selfaim.msg import aim

class custompoint2Pointcloud2:
    def __init__(self) -> None:
        rospy.init_node("custompoint2scan_node")
        self.position_filter = PositionFilter()
        self.pub = rospy.Publisher("/cloud", PointCloud2, queue_size=1)  #发布的点云
        self.pub_split_cloud=rospy.Publisher("/split_cloud",PointCloud2,queue_size=5)
        self.pub_target_cloud=rospy.Publisher("/target_cloud",PointCloud2,queue_size=5)
        self.pub_yaw=rospy.Publisher("/aim_target",aim,queue_size=20)
        self.rate=rospy.Rate(20)
        self.yaw_msg=aim()

        self.target=[[2.275,3.7],  #左一型0
                     [2.275,0.5],  #中一型1
                     [2.275,-2.7],  #右一型2
                     [4.175,1.840],  #左前二型3
                     [4.175,-0.8],  #右前二型4
                     [5.475,0.5],  #a柱5
                     [6.775,1.840],  #左后二型6
                     [6.775,-0.8]]  #右后二型7
        
        self.befor_index=[]
        self.before_ordemtry=[]  #记录最近一次8个柱子的偏航和当前的ordemtry
        
        
        # self.target=[[1.775,3.7],  #左一型
        #              [1.775,0.5],  #中一型
        #              [1.775,-2.7],  #右一型
        #              [3.675,1.840],  #左前二型
        #              [3.675,-0.8],  #右前二型
        #              [4.975,0.5],  #a柱
        #              [6.275,1.840],  #左后二型
        #              [6.275,-0.8]]  #右后二型
        
        # self.target=[[2.15,3.04],  #左一型
        #              [2.29,0.39],  #中一型
        #              [2.31,-2.75],  #右一型
        #              [4.17,1.74],  #左前二型
        #              [4.18,-0.85],  #右前二型
        #              [5.45,0.41],  #a柱
        #              [6.74,1.72],  #左后二型
        #              [6.72,-0.81]]  #右后二型
        self.search_radius = 0.30  # 匹配点云的限制距离
        self.before_yaw=[]  #记录上一次的yaw
        self.limit_yaw=0.01  #当我们的yaw差的值小于这个数就继承上一次的yaw
        self.compensation=0.05 #对于距离上的补偿
        self.aim_distance=1.8  #点云识别方法的阈值
        self.yaw_diff_long = 0.5/180*np.pi  #对于偏航的补偿
        self.yaw_diff_short = 1.2/180*np.pi  #对于偏航的补偿
        self.y_tolerate = 0.01  #对于点云点的y轴补偿
        self.tolerate_yaw = 0.02  #对于偏航的tolerate
        
        while not rospy.is_shutdown():
            msg=rospy.wait_for_message("/livox/lidar",CustomMsg,timeout=None)  #CustomMsg的msg
            msg_odometry=rospy.wait_for_message("/Odometry", Odometry,timeout=None)  #Odometry的msg

            self.pos_filtered=self.Odometry_callback(msg_odometry)  #得到lidar的相对于camera_init的坐标和偏航

            self.cloud_msg,self.cloud_points=self.CustomMsg2PointCloud2_callback(msg)  #得到二维点云和带分割的点云
            # self.cloud_points=self.CustomMsg2PointCloud2_callback(msg)  #得到二维点云和带分割的点云

            self.pub.publish(self.cloud_msg)

            self.trans_target=self.point_trans()  #转换了坐标之后的target

            if len(self.cloud_points)>0:  #发布分割点云
                # 匹配点云
                self.target_point=self.match_cloud()

                self.target_cloud_msg=self.ros2PointCloud2(self.target_point)
                self.pub_target_cloud.publish(self.target_cloud_msg)

                #求得偏航
                self.yaw_group, self.dis_group=self.caculate_yaw()
                self.yaw_msg.yaw=self.yaw_group
                self.yaw_msg.dis=self.dis_group
                self.pub_yaw.publish(self.yaw_msg)
                self.rate.sleep()

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
    
    '''
    def CustomMsg2PointCloud2_callback(self,msg:CustomMsg):
         # 获得CustomMsg中的CustomPoint
        custompoint=msg.points
        cloud_points=[]
        for i in custompoint:
            if 0.06<i.z<0.5:  #筛选中间层的点云
                # x = i.x + self.y_tolerate*np.cos(self.pos_filtered[2])
                # y = i.y + self.y_tolerate*np.sin(self.pos_filtered[2])
                x=i.x
                y=i.y
                z = 1.0

                if (not math.isinf(x)) and (not math.isinf(y)):
                    yaw=atan2(y,x)
                    cloud_points.append([x,y,yaw])
       
        cloud_points=sorted(cloud_points,key=lambda x:x[2])
        cloud_points=[[x[0],x[1]] for x in cloud_points]
        
        return cloud_points
    '''

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

        # for i in self.target:
            # i[0]+=self.y_tolerate*np.cos(self.pos_filtered[2]+np.pi/2)
            # i[1]+=self.y_tolerate*np.sin(self.pos_filtered[2]+np.pi/2)

        for i in custompoint:
            if 0.07<i.z<0.35:  #筛选中间层的点云
                # x = i.x + self.y_tolerate*np.cos(self.pos_filtered[2]+np.pi/2)
                # y = i.y + self.y_tolerate*np.sin(self.pos_filtered[2]+np.pi/2)
                x=i.x
                y=i.y
                z = 1.0
                points_data.append(struct.pack('fff', x, y, z))

                if (not math.isinf(x)) and (not math.isinf(y)):
                    yaw=atan2(y,x)
                    cloud_points.append([x,y,yaw])
                width+=1
        cloud_msg.data = b''.join(points_data)
        
        cloud_points=sorted(cloud_points,key=lambda x:x[2])
        cloud_points=[[x[0],x[1]] for x in cloud_points]

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
 
    def match_cloud(self):  #匹配点云
        target_point=[]

        count=0

        cloud_points=np.array(self.cloud_points)
        # 构建 KDTree
        tree = cKDTree(cloud_points)
        
        for ttarget in self.trans_target:
            # 查询 对应柱子坐标 半径为 0.3m 的圆内的所有坐标点
            points=[]
            query_point = np.array(ttarget)
            radius = self.search_radius
            indices = tree.query_ball_point(query_point, r=radius)
            if not indices:
                points.append(ttarget)
            else:
                count+=1
                for i in indices:
                    points.append(self.cloud_points[i])
            target_point.append(points)

        # print("匹配命中率:",count/len(self.trans_target)*100,"%")
        return target_point
        
    def caculate_yaw(self):  #计算偏航
        mid_point=[]
        dis_group=[]
        current_ordemtry=self.pos_filtered[2]
        is_one_point=[]
        
        for i in self.target_point:
            #近一点的柱子用取中值的方法    
            if np.sqrt(i[0][0]**2+i[0][1]**2) <= self.aim_distance:
                mid = int(len(i)/2)
                distance=np.sqrt(i[mid][0]**2+i[mid][1]**2)+self.compensation
                mid_point.append(i[mid])
            #远一点的柱子用最近点的匹配
            else:
                min_point=min(i, key= lambda x: fabs(x[0])+fabs(x[1]))
                distance=np.sqrt(min_point[0]**2+min_point[1]**2)+self.compensation
                mid_point.append(min_point)
            
            dis_group.append(distance)

            if len(i)==1:  #记录没有找到点云的点云分组
                is_one_point.append(self.target_point.index(i))        
           
        yaw_group=[]
        #计算偏航
        for i in mid_point:
            rad=atan2(i[1],i[0])
            index=mid_point.index(i)    #对于每个柱子对应的下标进行访问
            offset=0
            offset= 0.03/3.726*dis_group[index]  #根据柱子的距离给一个偏航的补偿
            # rad=atan2(i[1],i[0])
            # if dis_group[mid_point.index(i)] >= self.aim_distance:
            #     # offset = self.yaw_diff_long
            #     offset = 0.02
            # else:
            #     # offset = self.yaw_diff_short
            #     offset = -0.02

            # if mid_point.index(i) in self.left_index:
            #     offset = -offset
            # elif mid_point.index(i) in self.mid_index:
            #     offset = -0.02

            if index in is_one_point and self.before_ordemtry:  #如果对应柱子的点云没有搜索到点，那么就用通过之前的偏航进行计算
                rad = self.before_ordemtry[index][0]+(current_ordemtry-self.before_ordemtry[index][1])
                self.before_ordemtry[index][0]=rad
                self.before_ordemtry[index][1]=current_ordemtry
            elif index not in is_one_point and self.before_ordemtry:     #如果搜索到了柱子的点云，那么更新before_ordemtry        
                self.before_ordemtry[index][0]=rad
                self.before_ordemtry[index][1]=current_ordemtry

            rad += offset

            yaw_group.append(rad)

        #是否继承上一次的偏航
        if self.before_yaw:
            for i in range(len(yaw_group)):
                if fabs(yaw_group[i]-self.before_yaw[i]) <= self.limit_yaw and fabs(
                    self.before_yaw[i]) <= self.tolerate_yaw:
                    yaw_group[i]=self.before_yaw[i]

        if not self.before_ordemtry:  #等待第一次偏航算完就进行初始化 before_ordemtry
            # self.before_ordemtry.append([x,current_ordemtry] for x in yaw_group)
            for x in yaw_group:
                self.before_ordemtry.append([x,current_ordemtry])

        self.before_yaw=yaw_group

        return yaw_group,dis_group

class PositionFilter:  #卡尔曼滤波
    def __init__(self):
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = [0, 0, 0]  # 初始状态量
        self.kf.H = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])  # 测量矩阵
        self.kf.P *= 1000  # 初始协方差矩阵
        self.kf.R = np.diag([0.1, 0.1, 0.1])  # 测量噪声的协方差矩阵
        self.kf.Q = np.eye(3) * 0.05 # 过程噪声的协方差矩阵

    def update(self, z):
        self.kf.predict()
        self.kf.update(z)
        return self.kf.x

if __name__=="__main__":
    last_update_time = None
    ekf_pos = None
    ekf_cov = None
    vision_pos = None
    custompoint2Pointcloud2()
