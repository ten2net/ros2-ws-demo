import string
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from custom_msg_cpp.msg import AmazingQuote

from custom_msg.my_custom_msg import SampleClass, sample_function_for_square_of_sum
# from entity_controller.msg import Person

class TurtleNode(Node):
    def __init__(self):
        super().__init__('TurtleNode')  # 使用固定的节点名称以避免与launch文件中的参数冲突
        
        # msg = Person()
        # msg.name = "ROS User"
        # msg.age = 4  
        # print(">>>>>>>>>>>>>",msg)      
        # 声明参数
        self.declare_parameter('node_name', 'TurtleNode')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.2)
        # 获取参数
        self.node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        self.get_logger().info(f">>>>>>>>{self.node_name} {self.linear_speed} {self.angular_speed}")
        
        # 使用参数设置发布者和订阅者
        self.publisher_ = self.create_publisher(Twist, f'{self.node_name}/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            f'{self.node_name}/pose',
            self.listener_callback,
            10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        a: float = random.uniform(0, 1)
        b: float = random.uniform(1, 2)
        c: float = sample_function_for_square_of_sum(a, b)
        self.get_logger().info(f'sample_function_for_square_of_sum({a},{b}) returned {c}.')

        random_name_ascii: str = ''.join(random.choice(string.ascii_letters) for _ in range(10))
        sample_class_with_random_name = SampleClass(name=random_name_ascii)
        self.get_logger().info(f'sample_class_with_random_name.get_name() '
                               f'returned {sample_class_with_random_name.get_name()}.')        
        
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"{self.node_name} {msg.linear.x} {msg.linear.z}")

    def listener_callback(self, msg):
        self.get_logger().info('Turtle position: x=%f, y=%f, theta=%f' % (msg.x, msg.y, msg.theta))

def main(args=None):
    rclpy.init(args=args)
    turtlenode = TurtleNode()
    rclpy.spin(turtlenode)
    turtlenode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()