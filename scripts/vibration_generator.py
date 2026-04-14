import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from random import randint

class VibrationGenerator(Node):
    def __init__(self):
        super().__init__('vibration_generator')

        self.publisher = self.create_publisher(JointState,'/joint_states',10)
        self.timer = self.create_timer(0.01,self.publish_vibration)

        self.counter = 0
        self.get_logger().info('Vibration generator started!')
    def publish_vibration(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["plate_joint"]
        msg.position = [0.005*randint(-10,10)]
        msg.velocity = [0.1*randint(-2,2)]
        msg.effort = [5]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VibrationGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()