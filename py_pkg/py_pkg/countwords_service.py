#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.srv import CountWords

class CountWordsServiceNode(Node):
    def __init__(self):
        super().__init__("count_words_server")
        self.count_words_service = self.create_service(CountWords, "count_words", self.callback_count_words)
        self.get_logger().info("Word counter has been started.")
    def callback_count_words(self, request, response): #request: CountWords.Request, response: CountWords.Response
        self.get_logger().info("Received words: " + request.words)
        response.count = len(request.words.split(' '))
        return response
def main(args=None):
    rclpy.init(args=args)
    node = CountWordsServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()