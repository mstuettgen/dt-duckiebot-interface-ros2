import time
import sys

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.dagu_car_include.dagu_wheels_driver import DaguWheelsDriver


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(WheelsCmdStamped, '/wheels_cmd', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = 5.0
        msg.vel_right = 5.0
        self.publisher_.publish(msg)
#        self.get_logger().info('"Publishing: vel_left: %d | vel_right %d"' , msg.vel_left, msg.vel_right)
#        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
