#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import nengo
import random
import matplotlib.pyplot as plt
class SNNNavigator(Node):
    def __init__(self):
        super().__init__('snn_navigator')
        
        # ROS setup
        self.cmd_vel_pub = self.create_publisher(TwistStamped,'cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan,'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image,'/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        
        # Sensor data
        self.imu_variance = 0.0
        self.recent_imu_z = []
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.red_intensity = 0.0
        
        # Navigation
        self.state = 'FORWARD'
        self.turn_duration = 0
        self.turn_direction = 1
        self.steps_since_turn = 0
        self.next_random_turn = random.randint(50, 150)
        
        # Learning
        self.weight = 0.1 # CS→Hidden weight
        self.learning_rate = 0.001
        
        # Data recording
        self.history = {'time': [], 'weight': [], 'red': [],
        'vib': [], 'avoid': []}
        self.time_step = 0
        
        # Build SNN
        self.build_snn()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Started! Initial weight: {self.weight:.3f}')
        
    def build_snn(self):
        """Simple SNN: CS + US → Hidden → Avoidance"""
        self.model = nengo.Network() 
        with self.model:
            # LIF neurons
            self.cs = nengo.Ensemble(30, 1, neuron_type=nengo.LIF())
            self.us = nengo.Ensemble(30, 1, neuron_type=nengo.LIF())
            self.hidden = nengo.Ensemble(100, 1, neuron_type=nengo.LIF())
            self.avoid = nengo.Ensemble(50, 1, neuron_type=nengo.LIF())
            
            # Inputs
            cs_in = nengo.Node(lambda t: self.red_intensity / 100.0)
            us_in = nengo.Node(lambda t: 1.0 if self.imu_variance > 2.0 else 0.0)
            nengo.Connection(cs_in, self.cs)
            nengo.Connection(us_in, self.us)
            
            # Connections
            self.cs_conn = nengo.Connection(self.cs, self.hidden, transform=self.weight, synapse=0.1)
            nengo.Connection(self.us, self.hidden, transform=1.0, synapse=0.05)
            nengo.Connection(self.hidden, self.avoid, synapse=0.1)
            
            # Probes
            self.cs_probe = nengo.Probe(self.cs.neurons, synapse=0.01)
            self.us_probe = nengo.Probe(self.us.neurons, synapse=0.01)
            self.avoid_probe = nengo.Probe(self.avoid.neurons, synapse=0.01)
            
        self.sim = nengo.Simulator(self.model, dt=0.001)
        
    def scan_callback(self, msg):
        """Process Lidar data to detect obstacles"""
        
        # LaserScan gives 360 degree view
        ranges = np.array(msg.ranges)
        
        # Replace inf values with max range
        ranges[np.isinf(ranges)] = msg.range_max
        
        # Divide scan into sectors
        num_readings = len(ranges)
        
        # Front sector (±15 degrees around 0)
        front_start = 0
        front_end = int(num_readings * 15 / 360)
        front_start_back = int(num_readings * 345 / 360)
        front_readings = np.concatenate([ranges[front_start:front_end], ranges[front_start_back:]])
        self.front_distance = np.min(front_readings) if len(front_readings) > 0 else float('inf')
        
        # Left sector (60-120 degrees)
        left_start = int(num_readings * 60 / 360)
        left_end = int(num_readings * 120 / 360)
        left_readings = ranges[left_start:left_end]
        self.left_distance = np.mean(left_readings) if len(left_readings) > 0 else float('inf')
        
        # Right sector (240-300 degrees)
        right_start = int(num_readings * 240 / 360)
        right_end = int(num_readings * 300 / 360)
        right_readings = ranges[right_start:right_end]
        self.right_distance = np.mean(right_readings) if len(right_readings) > 0 else float('inf')
        
    def imu_callback(self, msg):
        """Process IMU data to detect vibration"""
        
        z_accel = msg.linear_acceleration.z
        
        # Store recent IMU readings
        self.recent_imu_z.append(z_accel)
        if len(self.recent_imu_z) > 50:
            self.recent_imu_z.pop(0)
            
        # Calculate variance (measure of vibration)
        if len(self.recent_imu_z) >= 10:
            mean_z = sum(self.recent_imu_z) / len(self.recent_imu_z)
            variance = sum((x - mean_z) ** 2 for x in
            self.recent_imu_z) / len(self.recent_imu_z)
            self.imu_variance = variance
            
    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,100,100]),np.array([10,255,255])),cv2.inRange(hsv, np.array([160,100,100]),np.array([180,255,255])))
            self.red_intensity = np.sum(mask > 0) / mask.size * 100.0
        except:
            pass
        
    def control_loop(self):
        # Run SNN and learn
        self.sim.run(0.1)
        cs_fire = np.sum(self.sim.data[self.cs_probe][-1]) > 5
        us_fire = np.sum(self.sim.data[self.us_probe][-1]) > 5
        avoid_strength = np.mean(np.sum(self.sim.data[self.avoid_probe][-10:], axis=1))
        
        # Hebbian learning
        if self.red_intensity > 10 and self.imu_variance > 2.0:
            self.weight += self.learning_rate * (self.red_intensity/100.0)
            self.weight = min(self.weight, 1.5)
            self.cs_conn.transform = self.weight
            self.get_logger().info(f'LEARNING! Weight:{self.weight:.3f}')
            
        # Record data
        self.time_step += 1
        self.history['time'].append(self.time_step * 0.1)
        self.history['weight'].append(self.weight)
        self.history['red'].append(self.red_intensity)
        self.history['vib'].append(1.0 if self.imu_variance > 2.0 else 0.0)
        self.history['avoid'].append(avoid_strength)
            
        # Navigation logic
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Priority 1: Vibration
        if self.imu_variance > 40.0 and self.state == 'FORWARD':
            self.turn_direction = 1 if (self.left_distance > self.right_distance) else -1
            self.state = 'REVERSE'
            self.turn_duration = 15
            
        # Priority 2: Learned avoidance 
        elif self.weight >= 0.5 and self.red_intensity > 10 and self.state == 'FORWARD': 
            self.turn_direction = 1 if self.left_distance > self.right_distance else -1
            self.state = 'TURN'
            self.turn_duration = 15
            self.get_logger().info('LEARNED AVOIDANCE!')
            
        # Priority 3: Obstacle
        elif self.front_distance < 1.5 and self.state == 'FORWARD':
            self.turn_direction = 1 if self.left_distance > self.right_distance else -1
            self.state = 'TURN'
            self.turn_duration = 15
            
            # Execute state
        if self.state == 'FORWARD':
            cmd.twist.linear.x = 1.0
            self.steps_since_turn += 1
            if self.steps_since_turn >= self.next_random_turn:
                self.state = 'TURN'
                self.turn_direction = random.choice([-1, 1])
                self.turn_duration = random.randint(5, 30)
                self.steps_since_turn = 0
                self.next_random_turn = random.randint(150, 300)
            
        elif self.state == 'REVERSE':
            cmd.twist.linear.x = -0.8
            self.turn_duration -= 1
            if self.turn_duration <= 0:
                self.state = 'TURN'
                self.turn_duration = 15
            
        elif self.state == 'TURN':
            cmd.twist.angular.z = 1.0 * self.turn_direction
            self.turn_duration -= 3
            if self.turn_duration <= 0:
                self.state = 'FORWARD'
                self.steps_since_turn = 0
                
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Executing state: {self.state}')

                
    def plot_learning(self):
        fig, axes = plt.subplots(4, 1, figsize=(12, 10)) 
        axes[0].plot(self.history['time'], self.history['weight'], 'b-', linewidth=2)
        axes[0].set_ylabel('CS→Hidden Weight')
        axes[0].set_title('Associative Learning: Weight Evolution')
        axes[0].grid(True)
        axes[1].plot(self.history['time'], self.history['red'], 'r-', linewidth=1)
        axes[1].set_ylabel('Red Color (%)')
        axes[1].set_title('Conditional Stimulus (CS)')
        axes[1].grid(True)
        axes[2].plot(self.history['time'], self.history['vib'], 'orange', linewidth=2)
        axes[2].set_ylabel('Vibration (0/1)')
        axes[2].set_title('Unconditional Stimulus (US)')
        axes[2].grid(True)
        axes[3].plot(self.history['time'], self.history['avoid'],'g-', linewidth=1)
        axes[3].set_ylabel('Avoidance Strength')
        axes[3].set_xlabel('Time (s)')
        axes[3].set_title('Learned Response')
        axes[3].grid(True)
        plt.tight_layout()
        plt.savefig('learning_curves.png', dpi=150)
        self.get_logger().info('Saved: learning_curves.png')
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    nav = SNNNavigator()
    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        pass
    nav.get_logger().info(f'Final weight: {nav.weight:.3f} (threshold: 0.5)')
    nav.get_logger().info('SUCCESS!' if nav.weight >= 0.5 else 'Need more training')
    nav.plot_learning()
    np.savez('data.npz', **nav.history)
    nav.cmd_vel_pub.publish(TwistStamped())
    nav.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()