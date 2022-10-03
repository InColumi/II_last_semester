#!/usr/bin/env python3
import rclpy
import random as rand
from rclpy.node import Node
from std_msgs.msg import String

class MyNode_listener(Node):

    def __init__(self):
        super().__init__(f"listener_{rand.randint(1, 90)}")
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.count_uniq_number = 5
        self.uniq_number = self.get_uniq_number(self.count_uniq_number)
        self.count_guess_numbers = 0
        self.get_logger().info(f"I have {self.uniq_number} numbers.")

    def listener_callback(self, msg):
        number = int(msg.data)
        if number in self.uniq_number:
            self.get_logger().info(f"+1, I guessed {number}")
            self.count_guess_numbers += 1
            if self.count_guess_numbers == self.count_uniq_number:
                self.get_logger().info("I win... Ueee")
        self.get_logger().info(f"I guessed {self.count_guess_numbers} numbers." )

    def get_uniq_number(self, size, count_tries=1000):
        answer = []
        counter = 1
        step = 0
        while(step < count_tries ):
            number = rand.randint(1, 10)
            if number not in answer:
                answer.append(number)
            if len(answer) == size:
                break
            step += 1
        if step >= count_tries:
            raise Exception('Not found uniq numbers')
        return answer

def main(args=None):
    rclpy.init(args=args)
    tacker = MyNode_listener()
    rclpy.spin(tacker)
    
    tacker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()