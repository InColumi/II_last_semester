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
        self.publisher_ = self.create_publisher(StringMessage, 'listener', 10)
        time_period = 1.0
        self.timer_ = self.create_timer(time_period, self.foo_publisher)
        self.create_setting()
        
        self.subscription_ = self.create_subscription(StringMessage, 'game', self.foo_subscription, 10)
        self.get_logger().info(f"My numbers: {self.get_name()} start: {self.number_for_guessing}")
    

    def create_setting(self):
        self.size_card = 3
        self.first_nubmer = 1
        self.last_nubmer = 10
        self.is_win = False
        self.number_for_guessing = [rand.randint(self.first_nubmer, self.last_nubmer) for i in range(1, self.size_card + 1)]


    def foo_subscription(self, msg: StringMessage):
        number = int(msg.data)
        while number in self.number_for_guessing:
            self.number_for_guessing.remove(number)
            self.get_logger().info(f":I have {number}")
        if self.number_for_guessing:
            self.get_logger().info(f"Remained: {self.number_for_guessing}")
        else:
            self.get_logger().info(f"Number is ended))))")
            self.is_win = True            

    
    def foo_publisher(self):
        if self.is_win:
            item = StringMessage()
            item.status_ = 1
            item.data = self.get_name()
            self.publisher_.publish(item)


def main(args=None):
    rclpy.init(args=args)
    game = Listener()
    rclpy.spin(game)
    
    game.destroy_node()