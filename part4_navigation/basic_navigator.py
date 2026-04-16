#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
import math
import random
import numpy as np
class BasicNavigator(Node):
    def __init__(self):
        super().__init__('basic_navigator')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped,'cmd_vel', 10)
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.imu_variance = 0.0
        self.recent_imu_z = []
        self.max_imu_samples = 50
        
        # Lidar data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.min_safe_distance = 1.5 # meters - stop if obstacle closer than this
        
        # Navigation parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.vibration_threshold = 20.0
        
        # State machine
        self.state = 'FORWARD' # States: FORWARD, TURN, REVERSE
        self.turn_duration = 0
        self.random_turn_direction = 1 # 1 for left, -1 for right
        
        # Random exploration parameters
        self.steps_since_last_turn = 0
        self.random_turn_interval = random.randint(50, 150) #Random wandering
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Basic Navigator Started!')
        self.get_logger().info('Speed: Linear={:.2f},Angular={:.2f}'.format(self.linear_speed, self.angular_speed))
        self.get_logger().info('Using LIDAR for obstacle detection')
        
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
        if len(self.recent_imu_z) > self.max_imu_samples:
            self.recent_imu_z.pop(0)
            
        # Calculate variance (measure of vibration)
        if len(self.recent_imu_z) >= 10:
            mean_z = sum(self.recent_imu_z) / len(self.recent_imu_z)
            variance = sum((x - mean_z) ** 2 for x in
            self.recent_imu_z) / len(self.recent_imu_z)
            self.imu_variance = variance
            
    def odom_callback(self, msg):
        """Track robot position"""
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
    def control_loop(self):
        """Main control loop - different strategies for vibration vs obstacles"""
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Priority 1: Check for vibration (highest priority)
        # Must REVERSE first to get off vibration plate, then turn
        
        if self.imu_variance > self.vibration_threshold:
            if self.state == 'FORWARD':
                self.get_logger().warn(f'VIBRATION DETECTED! Variance: {self.imu_variance:.2f}')
                self.get_logger().info(f'Distances - Left:{self.left_distance:.2f}m, Right: {self.right_distance:.2f}m')
                # Determine turn direction based on furthest distance
                
                if self.left_distance > self.right_distance:
                    self.random_turn_direction = 1 # Turn left
                    self.get_logger().info(f'Will turn LEFT toward open space ({self.left_distance:.2f}m)')
                else:
                    self.random_turn_direction = -1 # Turn right
                    self.get_logger().info(f'Will turn RIGHT toward open space ({self.right_distance:.2f}m)')
                    # First REVERSE to get off vibration plate
                    
                self.state = 'REVERSE'
                self.turn_duration = 15 # Reverse for 1.5 seconds
                
        # Priority 2: Check for obstacles using LIDAR
        # Can turn directly (no need to reverse)
        elif self.front_distance < self.min_safe_distance:
            if self.state == 'FORWARD':
                self.get_logger().info(f'OBSTACLE ahead at{self.front_distance:.2f}m!')
                self.get_logger().info(f'Distances - Left:{self.left_distance:.2f}m, Right: {self.right_distance:.2f}m')
                    
                # Turn toward the direction with FURTHEST distance
                if self.left_distance > self.right_distance:
                    self.random_turn_direction = 1 # Turn left
                    self.get_logger().info(f'Turning LEFT toward open space ({self.left_distance:.2f}m)')
                else:
                    self.random_turn_direction = -1 # Turn right
                    self.get_logger().info(f'Turning RIGHT toward open space ({self.right_distance:.2f}m)')
                        
                # Turn directly (no reverse needed)
                self.state = 'TURN'
                self.turn_duration = 15 # Turn ~90 degrees
                    
        # State machine logic
        if self.state == 'FORWARD':
            cmd.twist.linear.x = self.linear_speed
            cmd.twist.angular.z = 0.0
            # Random exploration: occasionally turn to explore
            self.steps_since_last_turn += 1
            if self.steps_since_last_turn >= self.random_turn_interval:
                self.random_turn_direction = random.choice([-1,1])
                turn_amount = random.choice(['SMALL', 'MEDIUM','LARGE'])
                if turn_amount == 'SMALL':
                    self.turn_duration = random.randint(5, 10)
                elif turn_amount == 'MEDIUM':
                    self.turn_duration = random.randint(10, 20)
                else: # LARGE
                    self.turn_duration = random.randint(20, 30)
                self.state = 'TURN'
                self.steps_since_last_turn = 0
                self.random_turn_interval = random.randint(50,150)
                self.get_logger().info(f'Random exploration turn({turn_amount})')
                
        elif self.state == 'REVERSE':
        # Reverse to get off vibration plate
            cmd.twist.linear.x = -self.linear_speed * 0.8
            cmd.twist.angular.z = 0.0
            self.turn_duration -= 1
            if self.turn_duration <= 0:
                # After reversing, now turn toward open space
                self.state = 'TURN'
                self.turn_duration = 15
                self.get_logger().info('Off vibration plate, now turning...')
                
        elif self.state == 'TURN':
            # Turn in place toward the furthest direction
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.angular_speed * self.random_turn_direction
            self.turn_duration -= 1
            if self.turn_duration <= 0:
                self.state = 'FORWARD'
                self.steps_since_last_turn = 0
                # Reset random interval for next exploration turn
                self.random_turn_interval = random.randint(50,150)
                self.get_logger().info('Turn complete, moving forward...')
                
        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    # Stop the robot
    navigator.cmd_vel_pub.publish(TwistStamped())
    navigator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()