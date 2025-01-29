#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.desired_distance_to_wall = 0.5  # m
        self.obstacle_distance_threshold = 0.3  # m
        self.turn_speed = 0.5  # Angular speed for turns (rad/s)
        self.forward_speed = 0.2  # Linear speed (m/s)
        self.follow_left_wall = True
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.angle_max = 0.0
        self.front_angle = np.pi / 16  # 10 degrees  
        self.first_read = False

    def lidar_callback(self, msg: LaserScan):
        
        # self.get_logger().info()
        if(not self.first_read):
           self.angle_min = msg.angle_min
           self.angle_max = msg.angle_max
           self.angle_increment = msg.angle_increment
        
        # Extract key distances from LiDAR scan
        front_indices = self.calculate_indices(-self.front_angle, +self.front_angle)
        left_indices = self.calculate_indices(self.angle_min, -self.front_angle) 
        right_indices = self.calculate_indices(self.front_angle, self.angle_max)

        # Extract ranges
        front_ranges = msg.ranges[front_indices[0]:front_indices[1]]
        left_ranges = msg.ranges[left_indices[0]:left_indices[1]]
        right_ranges = msg.ranges[right_indices[0]:right_indices[1]]

        front_range_min = min(front_ranges)
        front_range_max = max(front_ranges)
        left_range_min  =  min(left_ranges)
        left_range_max  =  max(left_ranges)
        right_range_min = min(right_ranges)
        right_range_max = max(right_ranges)

        print(front_range_min)
        print(front_range_max)
        # print(left_range_min )
        # print(left_range_max )
        # print(right_range_min)
        # print(right_range_max)
        # if (front_range_max) > 1.0:
        #     print(front_range_max)
        #     self.move_forward()
        # else:
        #     self.stop()
        
        # self.get_logger().info(f"Front ranges: {front_ranges}")
        # self.get_logger().info(f"Left ranges: {left_ranges}")
        # self.get_logger().info(f"Right ranges: {right_ranges}")
        

        
    
    def calculate_indices(self, start_angle, end_angle):
        start_index = int((start_angle - self.angle_min) / self.angle_increment)
        end_index = int((end_angle - self.angle_min) / self.angle_increment)
        return start_index, end_index


    def move_forward(self):
        
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def turn_left(self):
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.turn_speed
        self.cmd_vel_publisher.publish(twist)

    def turn_right(self):
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.turn_speed
        self.cmd_vel_publisher.publish(twist)

    def adjust_left(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.2  
        self.cmd_vel_publisher.publish(twist)

    def adjust_right(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = -0.2  
        self.cmd_vel_publisher.publish(twist)
    
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0  
        self.cmd_vel_publisher.publish(twist)
    


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()

    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

