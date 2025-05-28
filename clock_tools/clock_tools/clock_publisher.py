import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import time

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        clock_msg = Clock()
        clock_msg.clock = Time(sec=sec, nanosec=nanosec)
        self.publisher_.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
