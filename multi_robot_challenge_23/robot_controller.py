import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__(f'robot_controller')
        self.namespace = self.get_namespace()
        self.pub = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            LaserScan,
            f'{self.namespace}/scan',   # LIDAR topic in the self.namespace
            self.clbk_laser,
            10
        )
        self.sub = self.create_subscription(Int64, f'{self.namespace}/aruco_markers', self.clbk_marker, 10)
        self.get_logger().info(f"{self.namespace}")
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.front = False
        self.turning = False
        self.parallel = False
        self.move = True
        self.speed = 0.2
        self.angular = 0.0
        self.lidar_left_front = 100
        self.lidar_right_front = 100
        self.lidar_left_right = 100
        self.lidar_right_right = 100
        self.lidar_middle_right = 100
        self.lidar_left_left = 100
        self.lidar_right_left = 100
        self.lidar_middle_left = 100


    def clbk_marker(self, msg):
        self.get_logger().info(msg)

    def clbk_laser(self, msg):
        self.lidar_left_front = msg.ranges[20]
        self.lidar_right_front = msg.ranges[340]
        self.lidar_right_right = msg.ranges[260]
        self.lidar_middle_right = msg.ranges[270]
        self.lidar_left_right = msg.ranges[280]
        self.lidar_right_left = msg.ranges[80]
        self.lidar_middle_left = msg.ranges[90]
        self.lidar_left_left = msg.ranges[100]


    def timer_callback(self):
        if not self.move:
            return
        fl = self.lidar_left_front 
        fr = self.lidar_right_front 
        lr = self.lidar_left_right if self.namespace == "tb3_1" else self.lidar_left_left
        rr = self.lidar_right_right if self.namespace == "tb3_1" else self.lidar_right_left
        mr = self.lidar_middle_right if self.namespace == "tb3_1" else self.lidar_middle_left
        is_close = lr < 1 and mr < 1 and rr < 1
        is_parallel = abs(lr - mr) < 0.01*mr and abs(mr - rr) < 0.01*mr and is_close
        is_outer_corner = lr - mr > 0.1*mr and rr-mr > 0.1*mr
        distance = 0.65
        speed = 0.2
        angular = 0.4 if self.namespace == "tb3_1" else -0.4
        vel_msg = Twist()

        if self.parallel:
            if is_parallel or is_outer_corner:
                self.get_logger().info('paralleling ended')
                self.speed = speed
                self.angular = 0.0
                self.parallel = False
            else:
                self.get_logger().info('paralleling self')
                self.speed = 0.0
                self.angular = ((rr-lr)/mr)

        elif fl < distance or fr < distance:
            if self.turning:
                self.get_logger().info('corner turning ended')
                self.turning = False
            self.get_logger().info('front')
            self.speed = 0.0
            self.angular = angular
            self.front = True
        elif (abs(fl-fr) < 0.5 or fl > 2.0 and fr > 2.0) and self.front: # If bot is roughly parallel with wall in front
            # Maybe add fm sensor and check difference of fm-fl OR fm-fr
            self.get_logger().info('front turning ended')
            self.speed = speed
            self.angular = 0.0
            self.front = False
            self.parallel = True
        elif (rr < 1 or mr < 1) and lr > 2:
            self.get_logger().info('turning corner')
            self.turning = True
            self.speed = speed
            self.angular = -angular
        else:
            self.get_logger().info('nothing')
        vel_msg.linear.x = self.speed
        vel_msg.angular.z = self.angular

        vel_msg.angular.z = max(-0.4, min(0.4, vel_msg.angular.z))
        self.get_logger().info('left: ' + str(lr) + ' middle: ' + str(mr) + ' right: ' + str(rr) + ' frontleft: ' + str(fl) + ' frontright: ' + str(fr))
        self.get_logger().info('speed: ' + str(vel_msg.linear.x) + ' rotation: ' + str(vel_msg.angular.z))
        self.pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()