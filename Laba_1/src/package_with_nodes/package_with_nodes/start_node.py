#!/usr/bit/env python3

import rclpy
import random as rand
from rclpy.node import Node
from std_msgs.msg import String

class StringMessage(String):
    def __init__(self):
        super().__init__()
        self.status_ = 0


class Game(Node):
    def __init__(self):
        super().__init__(f"Publisher")
        self.send_numbers = self.create_publisher(StringMessage, 'game', 10)
        t = 1.0
        self.timer_ = self.create_timer(t, self.foo_publisher)
        self.start = 1
        self.end = 10
        self.numbers = [i for i in range(self.start, self.end + 1)]
        rand.shuffle(self.numbers)
        
        self.tacke_info = self.create_subscription(StringMessage, 'listener', self.foo_subscription, 10)
        self.get_logger().info(f"Start: {self.numbers}")


    def foo_subscription(self, msg: StringMessage):
        self.get_logger().info(f"{msg.data} is win!")
        self.timer_.cancel()
        
    
    def foo_publisher(self):
        item = StringMessage()
        if self.numbers:
            self.get_logger().info(f"Publisher: {self.numbers}")
            item.data = str(self.numbers.pop())
            self.send_numbers.publish(item)
            
        else:
            self.get_logger().info(f"Game is over!")
            self.timer_.cancel()
        


def main(args=None):
    rclpy.init(args=args)
    game = Game()
    rclpy.spin(game)
    
    game.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()