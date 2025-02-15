#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.srv import TimerControl
import time


class TimedCommandPublisher(Node):
    def __init__(self):
        super().__init__('timed_command_publisher')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('default_duration', 5.0)
        self.declare_parameter('default_linear_speed', 0.5)
        self.declare_parameter('default_angular_speed', 0.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.default_duration = self.get_parameter('default_duration').value
        self.default_linear_speed = self.get_parameter('default_linear_speed').value
        self.default_angular_speed = self.get_parameter('default_angular_speed').value

        self.command_pub = self.create_publisher(Twist, self.command_topic, 10) #Publisher des contrôles en vitesse Twist

        self.trigger_service = self.create_service(TimerControl, 'trigger_command', self.trigger_command_callback)

        self.is_active = False
        self.get_logger().info("TimedCommandPublisher ready. Call '/trigger_command' to start a command.")

    def trigger_command_callback(self, request, response):
        if self.is_active:
            response.success = False
            response.report = "A command is already active."
            self.get_logger().warn("Cannot start a new command. One is already active.")
            return response

        duration = request.duration if request.duration > 0 else self.default_duration
        linear_speed = request.linear_vx if request.linear_vx != 0 else self.default_linear_speed
        angular_speed = request.angular_wz if request.angular_wz != 0 else self.default_angular_speed

        self.send_command(duration, linear_speed, angular_speed)
        response.success = True
        response.message = "Command executed successfully."
        return response

    def send_command(self, duration, linear_speed, angular_speed):
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        self.get_logger().info(f"Publishing command for {duration:.2f} seconds.")
        self.is_active = True
        start_time = time.time()

        while time.time() - start_time < duration:
            self.command_pub.publish(cmd)
            time.sleep(0.1)  # 10 Hz

        self.stop_command()

    def stop_command(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0

        self.command_pub.publish(stop_cmd)
        self.get_logger().info("Command stopped.")
        self.is_active = False

def main(args=None):
    rclpy.init(args=args)
    node = TimedCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
