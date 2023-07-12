#!/usr/bin/env python3
import rospy
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from sensor_msgs.msg import  PointCloud2, PointField
import struct
import math
from tf.transformations import euler_from_quaternion
from filterpy.kalman import KalmanFilter
import numpy as np
import time
from math import fabs
from math import atan2

class touch_target_cloud:
    def __init__(self) -> None:

        self.pub_split_cloud=rospy.Publisher("/split_cloud",PointCloud2,queue_size=1)
        self.min_limitz = 0.6
        self.max_limitz = 1.6 #点云分割的z轴的跨越
        self.limit_dis= 0.05  #点云分割的阈值
        self.limit_xy = 0.1

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/livox/lidar",CustomMsg,timeout=10)  #CustomMsg的msg
            self.cloud_msg, self.cloud_points = self.CustomMsg2PointCloud2_callback(msg)

            # 分割点云
            self.point_group=self.split_cloud()
            self.split_cloud_msg=self.ros2PointCloud2(self.point_group)
            self.pub_split_cloud.publish(self.split_cloud_msg)


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
            if 0<i.z<1.9:
                x = i.x
                y = i.y
                z = i.z
                points_data.append(struct.pack('fff', x, y, 1.0))

                if (not math.isinf(x)) and (not math.isinf(y)):
                    yaw=atan2(i.y,i.x)
                    cloud_points.append([x,y,z,yaw])

                width+=1
        cloud_msg.data = b''.join(points_data)

        # 设置消息中的点数和是否包含无效点数据
        cloud_msg.width = width
        cloud_msg.height = 1
        cloud_msg.is_dense = True

        cloud_points=sorted(cloud_points,key=lambda x: x[3])
        cloud_points=[[x[0],x[1],x[2]] for x in cloud_points]

        return cloud_msg,cloud_points
    
    def split_cloud(self):  #分割点云的函数
        point_group=[]  #大组
        temp=[]  #小组
        for i in range(len(self.cloud_points)):
            if i == 0:
                temp.append(self.cloud_points[i])
            elif i ==  len(self.cloud_points)-1:
                temp.append(self.cloud_points[i])
                combine_before = False
                for j in point_group:
                    if fabs(j[-1][0]-temp[0][0])+ fabs(
                        j[-1][1]-temp[0][1]) <= self.limit_dis or np.sqrt((
                        j[-1][0]-temp[0][0])**2+(
                        j[-1][1]-temp[0][1])**2) <= self.limit_xy:

                        j.extend(temp)
                        combine_before = True
                        break
                
                if not combine_before:
                    point_group.append(temp)

            elif fabs(self.cloud_points[i][0]-temp[-1][0])+ fabs(
                self.cloud_points[i][1]-temp[-1][1]) <= self.limit_dis or np.sqrt((
                self.cloud_points[i][0]-temp[-1][0])**2+(
                self.cloud_points[i][1]-temp[-1][1])**2) <= self.limit_xy:

                temp.append(self.cloud_points[i])
            else:
                combine_before=False
                for j in point_group:
                    if fabs(j[-1][0]-temp[0][0])+ fabs(
                        j[-1][1]-temp[0][1]) <= self.limit_dis or np.sqrt((
                        j[-1][0]-temp[0][0])**2+(
                        j[-1][1]-temp[0][1])**2) <= self.limit_xy:

                        j.extend(temp)
                        combine_before = True
                        break
                
                if not combine_before:
                    point_group.append(temp)
                temp=[]
                temp.append(self.cloud_points[i])

        point_group = [x for x in point_group if x != []]

        if fabs(point_group[-1][-1][0]-point_group[0][0][0])+ fabs(
            point_group[-1][-1][1]-point_group[0][0][1]) <= self.limit_dis or np.sqrt((
            point_group[-1][-1][0]-point_group[0][0][0])**2+(
            point_group[-1][-1][1]-point_group[0][0][1])**2) <=self.limit_xy:

            point_group[0].extend(point_group[-1])
            del point_group[-1]
        
        # for i in len(point_group):

        group_z=[]
        for i in point_group:
            min_z = min(i,key= lambda x: x[2])
            max_z = max(i,key= lambda x: x[2])

            group_z.append(max_z[2]-min_z[2])
        
        new_point_group=[]
        for i in range(len(point_group)):
            if self.min_limitz <= group_z[i] <= self.max_limitz and 10<len(point_group[i]) <=80:
                new_point_group.append(point_group[i])
                print("group",i,"lenth:",len(point_group[i]))

        
        print("point_group lenth:",len(new_point_group))

        return new_point_group
    
    # def convergence_cloud(self):


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
                z = j[2] 
                points_data.append(struct.pack('fff', x, y, z))
                width+=1
        cloud_msg.data = b''.join(points_data)

        # 设置消息中的点数和是否包含无效点数据
        cloud_msg.width = width
        cloud_msg.height = 1
        cloud_msg.is_dense = True

        return cloud_msg


if __name__=="__main__":
    rospy.init_node("find_cloud_node")
    touch_target_cloud()