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
        self.publisher_ = self.create_publisher(StringMessage, 'game', 10)
        time_period = 1.0
        self.timer_ = self.create_timer(time_period, self.foo_publisher)
        self.create_setting()
        
        self.subscription_ = self.create_subscription(StringMessage, 'listener', self.foo_subscription, 10)
        self.get_logger().info(f"The game started: {self.unique_numbers}")
    
    def create_setting(self):
        self.size_card = 3
        self.first_nubmer = 1
        self.last_nubmer = 10
        self.unique_numbers = [i for i in range(self.first_nubmer, self.last_nubmer + 1)]
        rand.shuffle(self.unique_numbers)
        self.number_for_guessing = [rand.randint(self.first_nubmer, self.last_nubmer) for i in range(1, self.size_card)]

    def foo_subscription(self, msg: StringMessage):
        self.get_logger().info(f"{msg.data} is win!")
        self.timer_.cancel()
        
    
    def foo_publisher(self):
        item = StringMessage()
        if self.unique_numbers:
            self.get_logger().info(f"Publisher: {self.unique_numbers}")
            item.data = str(self.unique_numbers.pop())
            self.publisher_.publish(item)
            
        else:
            self.get_logger().info(f"Game is over! Number ended...")
            self.timer_.cancel()
        


def main(args=None):
    rclpy.init(args=args)
    game = Game()
    rclpy.spin(game)
    
    game.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()