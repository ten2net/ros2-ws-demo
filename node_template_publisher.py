#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class {{NODE_NAME}}Publisher(Node):
    def __init__(self):
        super().__init__('{{PKG_NAME}}_{{NODE_NAME}}')
        # 10: 这是发布者的质量服务（Quality of Service，QoS）设置。QoS设置定义了消息传递的可靠性和历史保持策略。在这里，10是QoS配置的整数表示，对应于rmw_qos_profile_sensor_data，这是一个适用于传感器数据的QoS配置，它提供了一定的可靠性保证和历史保持策略。
        self.publisher_ = self.create_publisher(String, '{{PKG_NAME}}/{{NODE_NAME}}', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS2!'
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        publisher = {{NODE_NAME}}Publisher()
        rclpy.spin(publisher)
        publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass        
    except Exception as e:
        print(f"Exception: {e}")

if __name__ == '__main__':
    main()    