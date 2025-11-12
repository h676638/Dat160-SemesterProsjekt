import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Leader(Node):
    def __init__(self):
        super().__init__('leader')
        self.create_subscription(Float32, '/tb3_1_test', self.callback_tb3_1, 10)
        self.create_subscription(Float32, '/tb3_2_test', self.callback_tb3_2, 10)

    def callback_tb3_1(self, msg):
        self.get_logger().info(f'[TB3_1] Value: {msg.data}')

    def callback_tb3_2(self, msg):
        print(f'[TB3_2] Value: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
