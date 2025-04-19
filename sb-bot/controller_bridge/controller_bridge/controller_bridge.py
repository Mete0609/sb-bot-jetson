import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
import math

class ControllerBridgeNode(Node):
    def __init__(self):
        super().__init__('controller_bridge_node')

        # 订阅控制器发布的整型速度指令（单位 pulses/10ms）
        self.left_cmd_sub = self.create_subscription(
            Int32, '/left_wheel_joint/command', self.left_cmd_callback, 10)
        self.right_cmd_sub = self.create_subscription(
            Int32, '/right_wheel_joint/command', self.right_cmd_callback, 10)

        # 发布目标电机脉冲速度（单位 pulses/10ms）
        self.left_speed_pub = self.create_publisher(Int32, 'target_left_speed', 10)
        self.right_speed_pub = self.create_publisher(Int32, 'target_right_speed', 10)

        # 订阅编码器速度反馈
        self.left_encoder_sub = self.create_subscription(
            Int32, 'left_wheel_speed', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(
            Int32, 'right_wheel_speed', self.right_encoder_callback, 10)

        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 发布 JointState
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 发布 odom + TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 小车参数（需与你实际参数一致）
        self.wheel_radius = 0.125  # meters
        self.wheel_base = 0.38  # meters（左右轮间距）
        self.encoder_to_rad = (2 * math.pi) / (60000 * 10)  # 60000 ticks/rev, 每10ms
        self.vel_to_pulse = 60000 / (2 * math.pi * self.wheel_radius * 100)  # m/s → pulse/10ms
        self.min_pulse = 150
        self.max_pulse = 300

        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz

    def left_cmd_callback(self, msg):
        self.left_speed_pub.publish(Int32(data=msg.data))

    def right_cmd_callback(self, msg):
        self.right_speed_pub.publish(Int32(data=msg.data))

    def left_encoder_callback(self, msg):
        self.left_velocity = msg.data * self.encoder_to_rad
        self.left_pos += self.left_velocity * 0.01

    def right_encoder_callback(self, msg):
        self.right_velocity = msg.data * self.encoder_to_rad
        self.right_pos += self.right_velocity * 0.01

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x * 0.01
        w = msg.angular.z

        v_l = v - w * self.wheel_base / 2
        v_r = v + w * self.wheel_base / 2

        left_pulse = int(v_l * self.vel_to_pulse)
        right_pulse = int(v_r * self.vel_to_pulse)

        if left_pulse != 0 and abs(left_pulse) < self.min_pulse:
            left_pulse = self.min_pulse if left_pulse > 0 else -self.min_pulse
        if right_pulse != 0 and abs(right_pulse) < self.min_pulse:
            right_pulse = self.min_pulse if right_pulse > 0 else -self.min_pulse

        if abs(left_pulse) > self.max_pulse:
            left_pulse = self.max_pulse if left_pulse > 0 else -self.max_pulse
        if abs(right_pulse) > self.max_pulse:
            right_pulse = self.max_pulse if right_pulse > 0 else -self.max_pulse

        self.get_logger().info(f"cmd_vel → L={left_pulse} R={right_pulse}")
        self.left_speed_pub.publish(Int32(data=left_pulse))
        self.right_speed_pub.publish(Int32(data=right_pulse))

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q

    def publish_joint_states(self):
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9

        v_left = self.left_velocity * self.wheel_radius
        v_right = self.right_velocity * self.wheel_radius
        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / self.wheel_base

        delta_x = v * dt * math.cos(self.th)
        delta_y = v * dt * math.sin(self.th)
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 发布 JointState
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pos, self.right_pos]
        msg.velocity = [self.left_velocity, self.right_velocity]
        self.joint_state_pub.publish(msg)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = self.euler_to_quaternion(0.0, 0.0, self.th)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # 发布 TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
