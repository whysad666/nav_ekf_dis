#!/usr/bin/env python3
import rospy
import random
import math
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geographiclib.geodesic import Geodesic

class TopicConverter:
    def __init__(self):
        rospy.init_node('topic_converter_node', anonymous=True)

        # 创建发布者
        self.pub_imu = rospy.Publisher('zh_origin', Float64MultiArray, queue_size=200)
        self.pub_gnss = rospy.Publisher('gnss_pv', Float64MultiArray, queue_size=10)
        self.pub_baro = rospy.Publisher('baro', Float64MultiArray, queue_size=10)
        self.pub_range = rospy.Publisher('range', Float64MultiArray, queue_size=10)
        self.d2r = 0.0174532925199
        self.cnt = 0
        self.llh = [0,0,0]
        self.speed_ned = [0,0,0]
        self.speed_ecef = np.array([[0],[0],[0]])

        self.baro_init = 86884
        self.baro_height_init = 500
        self.baro_height_now = 500

        # 订阅源话题并分别设置回调函数
        rospy.Subscriber('imu_zh', Imu, self.callback_imu)
        rospy.Subscriber('imu_origin', Float64MultiArray, self.callback_baro)
        rospy.Subscriber('gnss_origin', Float64MultiArray, self.callback_gnss)
        rospy.Subscriber('pos_vel', Float64MultiArray, self.callback_pos_vel)
        rospy.Subscriber('Distance_measurement/data_frame', Float64MultiArray, self.callback_dis)

        rospy.loginfo("Topic converter node initialized")
        rospy.spin()

    
    def callback_imu(self, msg):
        processed_imu = Float64MultiArray()
        processed_imu.data = [msg.header.stamp.to_sec(),0.05*self.cnt,msg.angular_velocity.x/200,msg.angular_velocity.y/200,msg.angular_velocity.z/200,msg.linear_acceleration.x/200,msg.linear_acceleration.y/200,msg.linear_acceleration.z/200]
        #processed_imu.data = [msg.header.stamp.to_sec(),0.05*self.cnt,msg.angular_velocity.x/200,msg.angular_velocity.y/200,msg.angular_velocity.z/200,msg.linear_acceleration.x/200,msg.linear_acceleration.y/200,msg.linear_acceleration.z/200]
        self.cnt+=1
        self.pub_imu.publish(processed_imu)
    
    def callback_gnss(self, msg):
        self.llh =[msg.data[3]*self.d2r,msg.data[2]*self.d2r,msg.data[4]]

    def baro_height_cal(self, baro_init, baro_now,baro_temp):
        """
        根据气压值计算相对高度
        使用国际标准大气模型 (ISA) 公式
        :param baro_init: 初始气压值 (Pa)
        :param baro_now: 当前气压值 (Pa)
        :return: 相对高度 (米)
        """

        # 使用气压高度公式计算两个气压值对应的高度
        # h = 18400×(1+tm/273​​)×log(P2/​P1​​)
        baro_height = 18400*math.log10(baro_init/baro_now)
        
        return baro_height
    
    def callback_baro(self, msg):
        processed_baro = Float64MultiArray()
        self.baro_height_now = self.baro_height_cal(self.baro_init,msg.data[22],msg.data[7])
        processed_baro.data = [msg.data[0],self.baro_height_now]
        self.pub_baro.publish(processed_baro)

    def callback_pos_vel(self, msg):
        processed_gnss = Float64MultiArray()
        C_en = np.array([
            [-np.sin(self.llh[0])*np.cos(self.llh[1]),-np.sin(self.llh[0])*np.sin(self.llh[1]),np.cos(self.llh[0])],
            [-np.sin(self.llh[1]),np.cos(self.llh[1]),0],
            [-np.cos(self.llh[0])*np.cos(self.llh[1]),-np.cos(self.llh[0])*np.sin(self.llh[1]),-np.sin(self.llh[0])]
        ])
        #C_en = C_en.transpose()
        self.speed_ecef= np.array([[msg.data[8]],[msg.data[9]],[msg.data[10]]])
        result = np.dot(C_en,self.speed_ecef)
        self.speed_ned[0] = result[0,0]
        self.speed_ned[1] = result[1,0]
        self.speed_ned[2] = result[2,0]
        processed_gnss.data = [msg.data[0],msg.data[17], self.llh[0], self.llh[1], self.llh[2],self.speed_ned[0],self.speed_ned[1],self.speed_ned[2]]


        #[msg.data[0],msg.data[1],msg.data[3]*self.d2r,msg.data[2]*self.d2r,msg.data[4]]
        self.pub_gnss.publish(processed_gnss)
        
    def geodesic_3d_distance(self,lat1, lon1, alt1, lat2, lon2, alt2):
    # 计算测地线距离
        geod = Geodesic.WGS84
        result = geod.Inverse(lat1, lon1, lat2, lon2)
        horizontal_distance = result['s12']  # 水平距离（米）
    # 计算垂直距离
        vertical_distance = abs(alt2 - alt1)
    # 计算3D距离
        distance_3d = math.sqrt(horizontal_distance**2 + vertical_distance**2)
        return distance_3d
    
    def callback_dis(self, msg):
        processed_dis = Float64MultiArray()
        range_sim = self.geodesic_3d_distance(msg.data[3], msg.data[4], msg.data[5], msg.data[9], msg.data[10], msg.data[11])
        processed_dis.data = [msg.data[0],msg.data[6],msg.data[9]*self.d2r,msg.data[10]*self.d2r ,msg.data[11],msg.data[8]]#range_sim+random.gauss(0,50)]
        self.pub_range.publish(processed_dis)

if __name__ == '__main__':
    try:
        TopicConverter()
    except rospy.ROSInterruptException:
        pass
