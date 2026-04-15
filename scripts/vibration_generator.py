import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class VibrationGenerator(Node):
    def __init__(self):
        super().__init__('vibration_generator')

        self.publisher = self.create_publisher(Float32,'/model/plate/joint/plate_joint/cmd_vel', 10)
        self.timer = self.create_timer(1,self.publish_vibration)

        self.counter = 0
        self.get_logger().info('Vibration generator started!')
    def publish_vibration(self):
        msg = Float32()
        msg.data = random.random()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VibrationGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()