import rclpy
from rclpy.node import Node
import serial
import threading

from std_msgs.msg import Int32

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        try:
            self.ser = serial.Serial('/dev/controll_serial', 115200, timeout=0.1)
            self.get_logger().info("串口连接成功: /dev/controll_serial @ 115200")
        except Exception as e:
            self.get_logger().error(f"串口初始化失败: {e}")
            exit(1)

        # 当前目标速度
        self.current_left_speed = 0
        self.current_right_speed = 0

        # 订阅目标速度指令
        self.left_speed_sub = self.create_subscription(
            Int32, 'target_left_speed', self.left_speed_callback, 10)
        self.right_speed_sub = self.create_subscription(
            Int32, 'target_right_speed', self.right_speed_callback, 10)

        # 发布编码器读取速度
        self.left_encoder_pub = self.create_publisher(Int32, 'left_wheel_speed', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'right_wheel_speed', 10)

        # 启动串口读取线程
        self.serial_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.serial_thread.start()

    def set_wheel_speeds(self, left_speed: int, right_speed: int):
        self.current_left_speed = left_speed
        self.current_right_speed = right_speed
        try:
            left_cmd = f"#L7={-left_speed}!"
            right_cmd = f"#R7={right_speed}!"  # 右轮方向取负
            # left_cmd = f"#L7={-left_speed}!"
            # right_cmd = f"#R7={right_speed}!"  # 右轮方向取负
            self.get_logger().info(f"发送命令: {left_cmd.strip()} | {right_cmd.strip()}")
            self.ser.write(left_cmd.encode())
            self.ser.write(right_cmd.encode())
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"串口写入失败: {e}")

    def left_speed_callback(self, msg):
        self.set_wheel_speeds(msg.data, self.current_right_speed)

    def right_speed_callback(self, msg):
        self.set_wheel_speeds(self.current_left_speed, msg.data)

    def read_serial_loop(self):
        buffer = ''
        while rclpy.ok():
            try:
                data = self.ser.read(self.ser.in_waiting or 1).decode(errors='ignore')
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    # 跳过仅为 'L' 的片段或其他不完整项
                    if len(line) < 5 or not line.startswith('L:'):
                        continue
                    # self.get_logger().info(f"收到串口数据: {repr(line)}")
                    self.parse_feedback(line)
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")

    def parse_feedback(self, line):
        # 示例格式：L:120,R:-90
        if 'L:' in line and 'R:' in line:
            try:
                parts = line.strip().split(',')
                if len(parts) != 2:
                    raise ValueError("数据字段数量不对")

                left_str = parts[0].split(':')
                right_str = parts[1].split(':')

                if len(left_str) != 2 or len(right_str) != 2:
                    raise ValueError("字段格式不对")

                left_val = int(left_str[1])
                right_val = int(right_str[1])

                # ✅ 发布时右轮取反
                # self.left_encoder_pub.publish(Int32(data=left_val))
                # self.right_encoder_pub.publish(Int32(data=-right_val))

                self.left_encoder_pub.publish(Int32(data=-left_val))
                self.right_encoder_pub.publish(Int32(data=right_val))

                # ✅ 打印信息
                if(left_val != 0 or right_val != 0):
                    self.get_logger().info(
                        f"📥 接收串口数据: L={left_val}, R={right_val} | 📤 发布到 bridge: L={-left_val}, R={right_val}"
                    )

            except Exception as e:
                self.get_logger().warn(f"解析错误: \"{line}\" -> {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()