#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.srv import CountWords

class ResetCounterClientNode(Node):
    def __init__(self):
        super().__init__("count_words_client")
        self.client_ = self.create_client(CountWords, "count_words")
    def call_count_words(self, sentence):
        request = CountWords.Request()
        request.words = sentence
        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_count_words_response)
    def callback_count_words_response(self, future):
        response = future.result()
        self.get_logger().info("Word count: " + str(response.count))
def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClientNode()
    sentence = input("Write a sentence:")
    node.call_count_words(sentence)
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()