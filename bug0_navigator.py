import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class TurtleBotBug2(Node):
    def __init__(self):
        super().__init__('turtlebot_bug2_navigation')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.initialized = False
        self.goal_x = None
        self.goal_y = None
        self.position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0

        self.obstacle_detected = False
        self.following_obstacle = False
        self.min_obstacle_distance = 0.25  # ระยะหลบสิ่งกีดขวาง
        self.escape_threshold = 0.6  # ระยะที่หุ่นสามารถกลับเข้าสู่เส้นทาง

        self.timer = self.create_timer(0.1, self.control_loop)

    def initialize_goal(self):
        """ กำหนดจุดหมายข้างหน้าหุ่น 4 เมตร ตามทิศที่หุ่นหันไป """
        if not self.initialized:
            self.goal_x = self.position['x'] + 4.0 * math.cos(self.yaw)
            self.goal_y = self.position['y'] + 4.0 * math.sin(self.yaw)
            self.initialized = True
            self.get_logger().info(f"Goal set to: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def odom_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.initialize_goal()

    def lidar_callback(self, msg):
        """ อ่านค่าจาก LiDAR """
        self.front_distance = min(min(msg.ranges[0:20] + msg.ranges[-20:]), 10)  # ด้านหน้า
        self.left_distance = min(min(msg.ranges[60:100]), 10)  # ด้านซ้าย
        self.right_distance = min(min(msg.ranges[260:300]), 10)  # ด้านขวา

        if self.front_distance < self.min_obstacle_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
            if self.following_obstacle and self.front_distance > self.escape_threshold and self.line_of_sight():
                self.following_obstacle = False  # กลับเข้าเส้นทางเดิม

    def control_loop(self):
        if not self.initialized:
            return

        twist = Twist()

        if self.reached_goal():
            self.get_logger().info('Goal reached! Stopping TurtleBot3.')
            self.publisher_.publish(Twist())  # หยุดการเคลื่อนที่
            return

        if self.obstacle_detected:
            self.get_logger().info('Obstacle detected! Avoiding...')
            self.following_obstacle = True
            twist.angular.z = 0.6  # หมุนเพื่อหลบสิ่งกีดขวาง
            twist.linear.x = 0.0
        elif self.following_obstacle:
            self.get_logger().info('Following obstacle boundary...')
            twist.linear.x = 0.25  # เดินตามแนวสิ่งกีดขวาง
        else:
            self.get_logger().info('Moving to goal...')
            self.align_with_goal(twist)

        self.publisher_.publish(twist)

    def align_with_goal(self, twist):
        """ หันไปยังเป้าหมายและเคลื่อนที่ """
        goal_angle = math.atan2(self.goal_y - self.position['y'], self.goal_x - self.position['x'])
        angle_diff = goal_angle - self.yaw

        if abs(angle_diff) > 0.1:
            twist.angular.z = 0.3 * (angle_diff / abs(angle_diff))  # ปรับทิศทาง
        else:
            twist.linear.x = 0.25  # เคลื่อนที่ตรงไปยังเป้าหมาย

    def line_of_sight(self):
        """ ตรวจสอบว่าไม่มีสิ่งกีดขวางด้านหน้า และสามารถกลับสู่เส้นทางเดิมได้ """
        return (self.front_distance > self.escape_threshold and
                self.left_distance > self.escape_threshold and
                self.right_distance > self.escape_threshold)

    def reached_goal(self):
        """ ตรวจสอบว่าหุ่นยนต์ถึงเป้าหมายหรือยัง """
        distance = math.sqrt((self.goal_x - self.position['x'])**2 + (self.goal_y - self.position['y'])**2)
        return distance < 0.1

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotBug2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
