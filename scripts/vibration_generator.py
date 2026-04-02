import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import math


class VibrationGenerator(Node):
    def __init__(self):
        super().__init__('vibration_generator')

        self.publisher = self.create_publisher(Wrench,'/vibrating_floor/gazebo_ros_force',10)
        self.timer = self.create_timer(0.01,self.publish_vibration)

        self.counter = 0
        self.get_logger().info('Vibration generator started!')
    def publish_vibration(self):
        msg = Wrench()
        # Create sinusoidal vibration in Z axis
        frequency = 10.0# Hz
        amplitude = 50.0# Force magnitude
        msg.force.z = amplitude * math.sin(2 * math.pi *
        frequency * self.counter * 0.01)
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = VibrationGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()