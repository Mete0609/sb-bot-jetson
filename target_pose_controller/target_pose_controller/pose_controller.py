import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, PoseStamped
import math
import time

class TargetPoseController(Node):
    def __init__(self):
        super().__init__('target_pose_controller')

        # 发布器
        self.back_pub = self.create_publisher(Int32, '/back_flag', 10)
        self.joint_pub = self.create_publisher(Int32, '/joint_start', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # 订阅器
        self.subscription = self.create_subscription(Int32, '/target_position', self.position_callback, 10)
        self.flag_sub = self.create_subscription(Int32, '/pose_adjust', self.flag_callback, 10)

        # 控制参数
        self.image_center_x = 360
        self.kp = 0.005
        self.ki = 0.0
        self.kd = 0.0005
        self.tolerance = 30
        self.max_angular_z = math.radians(10)

        # 状态变量
        self.latest_target_x = None
        self.control_enabled = False
        self.target_lost_start_time = None
        self.awaiting_joint_start = False
        self.joint_timer_start = None
        self.centered_start_time = None
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # 定时控制循环
        self.timer = self.create_timer(0.4, self.control_loop)

        # 状态标识符
        self.timer_start = None
        self.timer_running = False

    def flag_callback(self, msg):
        self.control_enabled = (msg.data == 1)
        self.get_logger().info(f"🟢 pose_adjust = {msg.data}，控制状态：{'启用' if self.control_enabled else '禁用'}")

    def position_callback(self, msg):
        self.latest_target_x = msg.data

    def send_nav_goal(self, distance):
        goal = PoseStamped()
        goal.header.frame_id = 'base_link'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = distance
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)
        self.get_logger().info(f"📤 发布导航目标：前进 {distance} 米")

    def control_loop(self):
        now = time.time()

        # 🔒 未启用 or 无目标，跳过
        if not self.control_enabled or self.latest_target_x is None:
            return

        ros_now = self.get_clock().now()
        dt = (ros_now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        # 🔴 检查目标丢失
        if self.latest_target_x == -1:
            if self.target_lost_start_time is None:
                self.target_lost_start_time = now
                self.get_logger().warn("🔍 开始目标丢失计时...")
            elif now - self.target_lost_start_time >= 15.0:
                self.get_logger().warn("📢 目标持续丢失15秒，发布 /back_flag = 1")
                self.back_pub.publish(Int32(data=1))
                self.target_lost_start_time = None
            else:
                self.get_logger().info("⏳ 目标丢失中...")
            return
        else:
            self.target_lost_start_time = None

        # 🌀 PID控制旋转
        error = self.image_center_x - self.latest_target_x
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_angular_z), -self.max_angular_z)

        # ⏱ 控制 0.1s，等待 0.4s
        twist = Twist()
        twist.angular.z = 0.0 if abs(error) < self.tolerance else output
        self.cmd_publisher.publish(twist)
        self.get_logger().info(f"🎯 Error: {error:.2f}, Output: {output:.3f} rad/s")
        time.sleep(0.1)
        self.cmd_publisher.publish(Twist())

        # ✅ 居中后执行完整流程（阻塞方式）
        if abs(error) < self.tolerance and not self.awaiting_joint_start:
            self.get_logger().info("🎯 检测到目标居中，开始持续检测...")

            stable_start = time.time()
            while time.time() - stable_start < 2.0:
                # 持续检测2秒是否仍然居中
                if abs(self.image_center_x - self.latest_target_x) > self.tolerance or self.latest_target_x == -1:
                    self.get_logger().warn("⚠️ 目标偏离或丢失，取消前进流程")
                    return
                time.sleep(0.2)  # 检测间隔（适当）

            # 居中稳定，执行前进
            self.get_logger().info("✅ 目标持续居中2秒，发布前进 0.4m")
            self.send_nav_goal(0.4)

            # 等待15秒（模拟前进完成）并发布计时信息
            self.get_logger().info("⏳ 正在等待5秒后发布 joint_start")

            # 启动计时器
            if not self.timer_running:
                self.timer_start = time.time()
                self.timer_running = True

            # 每秒发布一次计时信息
            while time.time() - self.timer_start < 10.0:
                elapsed_time = time.time() - self.timer_start
                if elapsed_time % 1 == 0:
                    self.get_logger().info(f"✅剩下抓取信号发布还剩 {5 - int(elapsed_time)} 秒...")

                time.sleep(1)

            # 发布 joint_start = 1
            self.joint_pub.publish(Int32(data=1))
            self.get_logger().info("✅ 发布 joint_start = 1")
            self.awaiting_joint_start = True  # 防止再次触发

            self.timer_running = False  # 停止计时器
            return  # 结束本轮控制

def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

