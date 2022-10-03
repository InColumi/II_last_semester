#!/usr/bin/env python3
import rclpy
import random as rand
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):

    def __init__(self):
        super().__init__(f"tacker_{rand.randint(1, 90)}")
        self.publishers_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_collback)
        self.i = 0

    def timer_collback(self):
        msg = String()
        number = rand.randint(1, 10)
        msg.data = f"{number}"
        self.get_logger().info(f"Current number: {number}, {self.i}")
        self.publishers_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    tacker = MyNode()
    rclpy.spin(tacker)
    
    tacker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()