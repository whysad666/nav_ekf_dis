#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

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

        # 订阅源话题并分别设置回调函数
        rospy.Subscriber('imu_zh', Imu, self.callback_imu)
        rospy.Subscriber('gnss_origin', Float64MultiArray, self.callback_gnss)
        rospy.Subscriber('Distance_measurement/data_frame', Float64MultiArray, self.callback_dis)
        
        rospy.loginfo("Topic converter node initialized")
        rospy.spin()
    
    def callback_imu(self, msg):
        processed_imu = Float64MultiArray()
        processed_imu.data = [msg.header.stamp.to_sec(),0.05*self.cnt,msg.angular_velocity.x/200,msg.angular_velocity.y/200,msg.angular_velocity.z/200,msg.linear_acceleration.x/200,msg.linear_acceleration.y/200,msg.linear_acceleration.z/200]
        self.cnt+=1
        self.pub_imu.publish(processed_imu)
    
    def callback_gnss(self, msg):
        processed_gnss = Float64MultiArray()
        processed_gnss.data = [msg.data[0],msg.data[1],msg.data[3]*self.d2r,msg.data[2]*self.d2r,msg.data[4]]
        self.pub_gnss.publish(processed_gnss)
        processed_baro = Float64MultiArray()
        processed_baro.data = [msg.data[0],msg.data[4]-499]
        self.pub_baro.publish(processed_baro)
        
    def callback_dis(self, msg):
        # 获取原始距离值
        raw_distance = msg.data[8]
        # 应用低通滤波
        filtered_distance = self.low_pass_filter(raw_distance)
        processed_dis = Float64MultiArray()
        processed_dis.data = [msg.data[0],msg.data[6],msg.data[9],msg.data[10],msg.data[11],msg.data[8]]
        self.pub_range.publish(processed_dis)

if __name__ == '__main__':
    try:
        TopicConverter()
    except rospy.ROSInterruptException:
        pass
