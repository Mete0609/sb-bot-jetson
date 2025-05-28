import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import json

# 预设的目标位置表 (x, y, yaw)
GOAL_POSES = {
    '书桌': (0.8, 9.0, math.radians(180)),
    '茶几': (9.0, 5.0, math.radians(180)),
    '餐桌': (4.0, 6.0, math.radians(270)),
    '厨房': (0.66, 4.8, math.radians(180)),
}

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class NavBridgeNode(Node):
    def __init__(self):
        super().__init__('nav_bridge_node')
        self.navigator = BasicNavigator()

        self.recog_pub = self.create_publisher(Int32, '/target_recog_flag', 10)
        self.adjust_pub = self.create_publisher(Int32, '/pose_adjust', 10)

        # 订阅 back_flag = 1（新增）
        self.back_flag_sub = self.create_subscription(Int32, '/back_flag', self.back_callback, 10)

        self.origin_pose = self.create_pose(0.0, 0.0, 0.0)
        self.navigator.setInitialPose(self.origin_pose)

        self.navigator.waitUntilNav2Active()
        self.get_logger().info('导航系统已就绪，等待目标指令...')

        self.subscription = self.create_subscription(
            String,
            '/app_navigation_target',
            self.target_callback,
            10
        )

    def back_callback(self, msg):
        if msg.data == 1:
            self.get_logger().warn("🔁 收到 /back_flag=1，立即导航回原点！")
            self.adjust_pub.publish(Int32(data=0))  # 可选：提前关闭识别控制
            self.recog_pub.publish(Int32(data=0))
            self.return_to_origin()

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def target_callback(self, msg):
        try:
            control_data = json.loads(msg.data)
            action = control_data.get('action', None)
            parameters = control_data.get('parameters', {})
            target_name = parameters.get('position', None)

            self.get_logger().info(f'接收到指令: action={action}, position={target_name}')

            if action == "move" and target_name in GOAL_POSES:
                goal = GOAL_POSES[target_name]
                self.navigate_to_target_and_return(goal)
            else:
                self.get_logger().warn(f"未识别的动作或目标: action={action}, position={target_name}")

        except json.JSONDecodeError:
            self.get_logger().error('❌ 收到的消息不是有效JSON格式')
        except Exception as e:
            self.get_logger().error(f'❌ 处理指令异常: {e}')

    def navigate_to_target_and_return(self, goal):
        x, y, yaw = goal
        goal_pose = self.create_pose(x, y, yaw)

        self.get_logger().info(f'开始导航到: ({x:.2f}, {y:.2f})')
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            self.get_logger().info('🚗 正在前往目标...')
            rclpy.spin_once(self.navigator, timeout_sec=1.0)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ 已成功到达目标地点')

            self.get_logger().info('📤 发布识别请求标志: 1')
            self.recog_pub.publish(Int32(data=1))
            self.adjust_pub.publish(Int32(data=1))

        else:
            self.get_logger().error('❌ 导航失败或被取消')

    def return_to_origin(self):
        self.get_logger().info('🚗 返回原点中...')
        self.navigator.goToPose(self.origin_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=1.0)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ 成功返回原点')
        else:
            self.get_logger().error('❌ 返回原点失败')

def main(args=None):
    rclpy.init(args=args)
    node = NavBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
