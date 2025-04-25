import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import struct
import math
import tf_transformations

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.ser = serial.Serial('/dev/imu_serial', baudrate=115200, timeout=0.05)
        self.buffer = bytearray()

        self.latest_quat = None
        self.latest_gyro = None
        self.latest_accel = None

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        self.initial_quaternion = None
        self.zero_initialized = False
        self.allow_zeroing = False
        self.create_timer(1.0, self.enable_zeroing)

        self.timer = self.create_timer(0.02, self.read_serial)

    def enable_zeroing(self):
        self.allow_zeroing = True
        self.get_logger().info("🕒 允许记录 IMU 初始姿态为零点")

    def read_serial(self):
        try:
            new_data = self.ser.read_all()
            self.buffer.extend(new_data)

            while len(self.buffer) >= 5:
                if self.buffer[0] != 0x55 or self.buffer[1] != 0x55:
                    self.buffer.pop(0)
                    continue

                if len(self.buffer) < 4:
                    break

                frame_id = self.buffer[2]
                data_len = self.buffer[3]
                frame_len = 5 + data_len

                if len(self.buffer) < frame_len:
                    break

                frame = self.buffer[:frame_len]
                checksum = sum(frame[0:frame_len - 1]) & 0xFF
                actual = frame[frame_len - 1] & 0xFF

                if checksum == actual:
                    if frame_id == 0x01:
                        self.parse_imu_data(frame)
                    elif frame_id == 0x02:
                        self.parse_quaternion_data(frame)
                    elif frame_id == 0x03:
                        self.parse_gyro_accel_data(frame)
                else:
                    self.get_logger().warn(f"❌ 帧校验失败: ID=0x{frame_id:02X}, calc=0x{checksum:02X}, actual=0x{actual:02X}, raw={frame.hex()}")

                self.buffer = self.buffer[frame_len:]

        except Exception as e:
            self.get_logger().warn(f"读取或解析串口数据出错：{e}")

    def parse_imu_data(self, frame):
        roll = struct.unpack('<h', frame[4:6])[0] / 32768.0 * 180.0
        pitch = struct.unpack('<h', frame[6:8])[0] / 32768.0 * 180.0
        yaw = struct.unpack('<h', frame[8:10])[0] / 32768.0 * 180.0
        # self.get_logger().info(f"🎯 姿态角 => Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")

    def parse_quaternion_data(self, frame):
        if len(frame) != 13:
            return

        checksum = sum(frame[0:12]) & 0xFF
        actual = frame[12] & 0xFF
        if checksum != actual:
            self.get_logger().warn(f"❌ 四元数帧校验失败: calc=0x{checksum:02X}, actual=0x{actual:02X}, raw={frame.hex()}")
            return

        q0 = struct.unpack('<h', frame[4:6])[0] / 32768.0
        q1 = struct.unpack('<h', frame[6:8])[0] / 32768.0
        q2 = struct.unpack('<h', frame[8:10])[0] / 32768.0
        q3 = struct.unpack('<h', frame[10:12])[0] / 32768.0

        current_q = [q1, q2, q3, q0]

        if self.initial_quaternion is None and self.allow_zeroing and not self.zero_initialized:
            self.initial_quaternion = current_q
            self.zero_initialized = True
            self.get_logger().info("✅ 当前姿态已记录为初始零点")

        if self.initial_quaternion is not None:
            q_offset_inv = tf_transformations.quaternion_inverse(self.initial_quaternion)
            relative_q = tf_transformations.quaternion_multiply(current_q, q_offset_inv)
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(relative_q)
            #self.get_logger().info(f"相对零点姿态角: Roll={math.degrees(roll):.2f}°, Pitch={math.degrees(pitch):.2f}°, Yaw={math.degrees(yaw):.2f}°")
            self.latest_quat = Quaternion(x=relative_q[0], y=relative_q[1], z=relative_q[2], w=relative_q[3])
        else:
            self.latest_quat = Quaternion(x=current_q[0], y=current_q[1], z=current_q[2], w=current_q[3])

        self.publish_imu()

    def parse_gyro_accel_data(self, frame):
        if len(frame) != 17:
            return

        checksum = sum(frame[0:16]) & 0xFF
        actual = frame[16] & 0xFF

        if checksum != actual:
            self.get_logger().warn(f"❌ 陀螺仪/加速度帧校验失败: calc=0x{checksum:02X}, actual=0x{actual:02X}, raw={frame.hex()}")
            return

        ACC_FSR = 4.0
        GYRO_FSR = 2000.0

        ax = struct.unpack('<h', frame[4:6])[0]
        ay = struct.unpack('<h', frame[6:8])[0]
        az = struct.unpack('<h', frame[8:10])[0]
        gx = struct.unpack('<h', frame[10:12])[0]
        gy = struct.unpack('<h', frame[12:14])[0]
        gz = struct.unpack('<h', frame[14:16])[0]

        acc_x = ax / 32768.0 * ACC_FSR * 9.8
        acc_y = ay / 32768.0 * ACC_FSR * 9.8
        acc_z = az / 32768.0 * ACC_FSR * 9.8

        gyro_x = gx / 32768.0 * GYRO_FSR
        gyro_y = gy / 32768.0 * GYRO_FSR
        gyro_z = gz / 32768.0 * GYRO_FSR

        self.latest_accel = (acc_x, acc_y, acc_z)
        self.latest_gyro = (gyro_x, gyro_y, gyro_z)
        self.publish_imu()

    def publish_imu(self):
        if self.latest_quat is None or self.latest_gyro is None or self.latest_accel is None:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation = self.latest_quat
        imu_msg.angular_velocity.x = math.radians(self.latest_gyro[0])
        imu_msg.angular_velocity.y = math.radians(self.latest_gyro[1])
        imu_msg.angular_velocity.z = math.radians(self.latest_gyro[2])
        imu_msg.linear_acceleration.x = self.latest_accel[0]
        imu_msg.linear_acceleration.y = self.latest_accel[1]
        imu_msg.linear_acceleration.z = self.latest_accel[2]

        imu_msg.orientation_covariance = [0.02, 0.0, 0.0,
                                          0.0, 0.02, 0.0,
                                          0.0, 0.0, 0.02]
        imu_msg.angular_velocity_covariance = [0.05, 0.0, 0.0,
                                               0.0, 0.05, 0.0,
                                               0.0, 0.0, 0.05]
        imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                  0.0, 0.1, 0.0,
                                                  0.0, 0.0, 0.1]
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
