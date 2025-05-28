import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # 初始化串口通信
        try:
            self.serial_port = serial.Serial('/dev/joint_serial', baudrate=9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 无法打开串口: {e}")
            self.serial_port = None

        # 订阅 joint_start 话题
        self.joint_start_subscription = self.create_subscription(Int32, '/joint_start', self.joint_start_callback, 10)

    def joint_start_callback(self, msg: Int32):
        # 只在收到 joint_start = 1 时发送串口命令
        if msg.data == 1:
            self.get_logger().info("🟢 收到 joint_start = 1，执行串口指令")
            self.send_serial_command()

    def send_serial_command(self):
        if self.serial_port and self.serial_port.is_open:
            # 发送一个串口命令
            self.serial_port.write(b'\x55\x55\x05\x06\x01\x01\x00')
            self.get_logger().info("📤 串口命令已发送")
        else:
            self.get_logger().warn("⚠️ 串口未打开，无法发送命令")


def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
