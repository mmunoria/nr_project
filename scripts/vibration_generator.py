import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class VibrationGenerator(Node):
    def __init__(self):
        super().__init__('vibration_generator')

        self.publisher = self.create_publisher(Float64,"/model/joint_controller_demo/joint/j1/cmd_vel", 10)
        self.timer = self.create_timer(0.01,self.publish_vibration)

        self.counter = 0
        self.get_logger().info('Vibration generator started!')
    def publish_vibration(self):
        msg = Float64()
        msg.data = (random.randint(-30,30)*1.0)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VibrationGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()