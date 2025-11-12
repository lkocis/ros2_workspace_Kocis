import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
import time 
import math 

class DrawLana(Node): 
    def __init__(self): 
        super().__init__('draw_lana') 
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        time.sleep(1) 
        self.turn(math.radians(-85))
        self.draw_line(1) 
        self.turn(math.radians(85))
        self.draw_line(0.6)

        self.turn(math.radians(65))
        self.draw_line(1.1)
        self.turn(math.radians(-135))
        self.draw_line(0.5)
        self.turn(math.radians(-100))
        self.draw_line(0.3)
        self.turn(math.radians(175))
        self.draw_line(0.3)
        self.turn(math.radians(-70))
        self.draw_line(0.7)
        self.turn(math.radians(65))
        self.draw_line(0.3)

        self.turn(math.radians(85))
        self.draw_line(1)
        self.turn(math.radians(-135))
        self.draw_line(1.3)
        self.turn(math.radians(140))
        self.draw_line(1.1)
        self.turn(math.radians(-175))
        self.draw_line(1.1)
        self.turn(math.radians(85))
        self.draw_line(0.3)

        self.turn(math.radians(65))
        self.draw_line(1.1)
        self.turn(math.radians(-135))
        self.draw_line(0.5)
        self.turn(math.radians(-100))
        self.draw_line(0.3)
        self.turn(math.radians(175))
        self.draw_line(0.3)
        self.turn(math.radians(-70))
        self.draw_line(0.7)
        
        self.get_logger().info('Name completed!') 

    def draw_line(self, distance, speed=1.0):
        msg = Twist()
        msg.linear.x = speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0 
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.get_logger().info('Moving forward...')

        duration = distance / speed
        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)  
        
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Line drawn!')


    def turn(self, angle, speed=1.0):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = speed if angle > 0 else -speed

        duration = abs(angle / speed)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            self.get_logger().info('Turn turtle!')
                    
def main(args=None): 
    rclpy.init(args=args) 
    node = DrawLana() 
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown() 
                    
if __name__ == '__main__': 
    main()