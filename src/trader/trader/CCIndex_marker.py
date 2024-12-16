import string
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from custom_msg_cpp.msg import AmazingQuote
from std_msgs.msg import Float32

import pandas as pd
import akshare as ak

from custom_msg.my_custom_msg import SampleClass, sample_function_for_square_of_sum
# from entity_controller.msg import Person
from .collector.akshare_data_collector import AkshareDataCollector
from .pool.pool import AmountStockPool

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time

data_points = []

class CCIndexNode(Node):
    def __init__(self):
        super().__init__('CCIndexNode')  # 使用固定的节点名称以避免与launch文件中的参数冲突
        
        # 声明参数
        self.declare_parameter('node_name', 'CCIndexNode')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        # 获取参数
        self.node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # 使用参数设置发布者和订阅者
        # self.publisher_ = self.create_publisher(Float32, f'{self.node_name}/ccindex', 10)
        self.publisher_ = self.create_publisher(MarkerArray, f'{self.node_name}/ccindex', 10)
        # self.timer = self.create_timer(2, self.timer_callback)
        self.timer = self.create_timer(2, self.publish_line_chart)


    def timer_callback(self):
        msg = Float32()
        msg.data = self.compute_market_sentiment_index()  
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)

    def listener_callback(self, msg):
        self.get_logger().info('Turtle position: x=%f, y=%f, theta=%f' % (msg.x, msg.y, msg.theta))

    def publish_line_chart(self):
        global data_points

        # 模拟实时数据点
        new_point = Point()
        new_point.x = time.time()  # 使用时间戳作为x轴
        new_point.y = self.compute_market_sentiment_index()  
        new_point.z = 0.0

        # 添加新点到数据点列表
        data_points.append(new_point)

        # 限制数据点数量，以避免内存溢出
        max_points = 100
        if len(data_points) > max_points:
            data_points.pop(0)

        # 创建MarkerArray并填充数据点
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point in data_points:
            marker.points.append(point)

        marker_array.markers.append(marker)

        self.get_logger().info('Publishing line chart with %d points' % len(data_points))
        self.publisher_.publish(marker_array)

    def compute_market_sentiment_index(self) -> float:
        stockPool = AmountStockPool()
        df =stockPool.get_data_frame(cloumn_name="amount", k=300)       

        # 定义涨跌幅区间和权重
        bins = [-float('inf'), -0.1, -0.075, -0.05, -0.025, 0.0, 0.025, 0.05, 0.075, 0.1, float('inf')]
        weights = [-16, -8, -4, -2, -1, 1, 2, 4, 8, 16]

        df['segment'] = pd.cut(df['pct'] / 100, bins=bins, labels=weights, right=True)

        df['weighted_amount'] = df['amount'] * df['segment'].astype(float)
        # 计算实际加权因子
        actual_weighted_factor = df['weighted_amount'].sum()
    
        # 计算最大可能的加权因子
        max_weighted_factor = df['amount'].sum() * max(weights)

        # 计算强弱指数
        strength_index = round(actual_weighted_factor / max_weighted_factor,3)
        return strength_index

def main(args=None):
    rclpy.init(args=args)
    node = CCIndexNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
