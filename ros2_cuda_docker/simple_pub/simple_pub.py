import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

import torch


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')

        self.cuda_available = torch.cuda.is_available()
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = String()
        msg.data = f"Is cuda available: {str(self.cuda_available)}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()