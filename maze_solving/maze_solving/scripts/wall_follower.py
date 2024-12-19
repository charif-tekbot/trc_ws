#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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

    def lidar_callback(self, msg: LaserScan):
        # Extract key distances from LiDAR scan
        # TODO
        return

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

