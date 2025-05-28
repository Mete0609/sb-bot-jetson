import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math

class ControllerBridgeNode(Node):
    def __init__(self):
        super().__init__('controller_bridge_node')

        self.wheel_radius = 0.0625
        self.wheel_base = 0.38
        self.pulse_to_vel = (2 * math.pi * self.wheel_radius )/ 600.0
        self.vel_to_pulse = 600.0 / (2 * math.pi * self.wheel_radius)
        self.min_pulse = 150
        self.max_pulse = 600

        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.left_cmd_sub = self.create_subscription(
            Int32, '/left_wheel_joint/command', self.left_cmd_callback, 10)
        self.right_cmd_sub = self.create_subscription(
            Int32, '/right_wheel_joint/command', self.right_cmd_callback, 10)

        self.left_speed_pub = self.create_publisher(Int32, 'target_left_speed', 10)
        self.right_speed_pub = self.create_publisher(Int32, 'target_right_speed', 10)

        self.left_encoder_sub = self.create_subscription(
            Int32, 'left_wheel_speed', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(
            Int32, 'right_wheel_speed', self.right_encoder_callback, 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/controller_bridge/odom', 10)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # self.send_static_transforms()
        self.timer = self.create_timer(0.02, self.publish_joint_states)

    # def send_static_transforms(self):
    #     transforms = []
    #     for name, x in [
    #         ('left_wheel', -self.wheel_base / 2),
    #         ('right_wheel', self.wheel_base / 2)
    #     ]:
    #         tf = TransformStamped()
    #         tf.header.stamp = self.get_clock().now().to_msg()
    #         tf.header.frame_id = 'base_link'
    #         tf.child_frame_id = name
    #         tf.transform.translation.x = x
    #         tf.transform.translation.y = 0.0
    #         tf.transform.translation.z = 0.0
    #         q = self.euler_to_quaternion(0.0, math.pi/2, 0.0)
    #         tf.transform.rotation = q
    #         transforms.append(tf)

    #     tf_base = TransformStamped()
    #     tf_base.header.stamp = self.get_clock().now().to_msg()
    #     tf_base.header.frame_id = 'base_footprint'
    #     tf_base.child_frame_id = 'base_link'
    #     tf_base.transform.translation.x = 0.0
    #     tf_base.transform.translation.y = 0.0
    #     tf_base.transform.translation.z = 0.1
    #     tf_base.transform.rotation = self.euler_to_quaternion(0.0, 0.0, math.pi/2)
    #     transforms.append(tf_base)

    #     self.static_tf_broadcaster.sendTransform(transforms)

    def left_cmd_callback(self, msg):
        self.left_speed_pub.publish(Int32(data=msg.data))

    def right_cmd_callback(self, msg):
        self.right_speed_pub.publish(Int32(data=msg.data))

    def left_encoder_callback(self, msg):
        self.left_velocity = msg.data * self.pulse_to_vel
        self.left_pos += self.left_velocity 

    def right_encoder_callback(self, msg):
        self.right_velocity = msg.data * self.pulse_to_vel
        self.right_pos += self.right_velocity

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x 
        w = msg.angular.z

    # # 如果线速度为0但有角速度，就强制走弧线
    #     if abs(v) < 1e-3 and abs(w) > 1e-3:
    #         v = 0.1  # 或者你可以设置为 0.1，看效果

        # v_l = v + w * self.wheel_base / 2
        # v_r = v - w * self.wheel_base / 2

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

        v_left = self.left_velocity 
        v_right = self.right_velocity 
        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / self.wheel_base

        delta_x = v * dt * math.cos(self.th)
        delta_y = v * dt * math.sin(self.th)
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = ['base_link_to_left_wheel', 'base_link_to_right_wheel']
        msg.position = [self.left_pos, self.right_pos]
        msg.velocity = [self.left_velocity, self.right_velocity]
        self.joint_state_pub.publish(msg)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = '/odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = self.euler_to_quaternion(0.0, 0.0, self.th)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        odom.pose.covariance = [
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
        ]

        odom.twist.covariance = [
            1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2
        ]



        self.odom_pub.publish(odom)


        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_footprint'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation = q
        # self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
