#!/usr/bit/env python3

import rclpy
import random as rand
from rclpy.node import Node
from std_msgs.msg import String

class StringMessage(String):
    def __init__(self):
        super().__init__()
        self.status_ = 0

class Listener(Node):
    def __init__(self):
        super().__init__(f"listener_{rand.randint(1, 10)}")
        self.send_info = self.create_publisher(StringMessage, 'listener', 10)
        t = 1.0
        self.timer_ = self.create_timer(t, self.send_info_to_game)
        self.size_list = 10
        self.start = 1
        self.end = 90
        self.is_win = False
        self.guessing_list = [rand.randint(self.start, self.end) for i in range(1, self.size_list + 1)]
        
        self.take_info = self.create_subscription(StringMessage, 'game', self.check_numbers, 10)
        self.get_logger().info(f"My numbers: {self.get_name()} start: {self.guessing_list}")
    

    def check_numbers(self, msg: StringMessage):
        number = int(msg.data)
        while number in self.guessing_list:
            self.guessing_list.remove(number)
            self.get_logger().info(f":I have {number}")
        if self.guessing_list:
            self.get_logger().info(f"Remained: {self.guessing_list}")
        else:
            self.get_logger().info(f"End")
            self.is_win = True            

    
    def send_info_to_game(self):
        if self.is_win:
            item = StringMessage()
            item.status_ = 1
            item.data = self.get_name()
            self.send_info.publish(item)
            self.timer_.cancel()


def main(args=None):
    rclpy.init(args=args)
    game = Listener()
    rclpy.spin(game)
    
    game.destroy_node()