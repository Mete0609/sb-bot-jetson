#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class KeyboardDriveNode(Node):
    def __init__(self):
        super().__init__('keyboard_drive_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s
        self.print_help()
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                twist = Twist()
                if key == 'w':
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                elif key == 'x':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'q':
                    print("退出控制")
                    break
                else:
                    continue
                self.pub.publish(twist)
                self.get_logger().info(f"发送 /cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        except KeyboardInterrupt:
            pass

    def print_help(self):
        print("""
W/S 前进/后退
A/D 原地左转/右转
X   停止
Q   退出
        """)

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardDriveNode()
    rclpy.shutdown()

