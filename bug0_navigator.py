import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class TurtleBotBugAlgorithm(Node):
    def __init__(self):
        super().__init__('turtlebot_bug_navigation')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.goal_x = 4.0  # เป้าหมายที่ (4,0)
        self.goal_y = 0.0
        self.position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0
        self.obstacle_detected = False
        self.following_obstacle = False
        self.min_obstacle_distance = 0.35  # ระยะห่างขั้นต่ำ
        self.escape_threshold = 0.6  # ออกจากสิ่งกีดขวางที่ระยะห่างนี้
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        # คำนวณ yaw (มุมของหุ่นยนต์)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def lidar_callback(self, msg):
        self.front_distance = min(min(msg.ranges[0:20] + msg.ranges[-20:]), 10)  # หน้าตรง
        self.left_distance = min(min(msg.ranges[60:100]), 10)  # ซ้าย
        self.right_distance = min(min(msg.ranges[260:300]), 10)  # ขวา
        
        if self.front_distance < self.min_obstacle_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
            if self.following_obstacle and self.front_distance > self.escape_threshold and self.line_of_sight():
                self.following_obstacle = False  # กลับเข้าเส้นทางเดิม

    
    def control_loop(self):
        twist = Twist()
        
        if self.reached_goal():
            self.get_logger().info('Goal reached! Stopping TurtleBot3.')
            self.publisher_.publish(Twist())  # หยุด
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
        goal_angle = math.atan2(self.goal_y - self.position['y'], self.goal_x - self.position['x'])
        angle_diff = goal_angle - self.yaw
        
        if abs(angle_diff) > 0.1:
            twist.angular.z = 0.3 * (angle_diff / abs(angle_diff))  # ปรับทิศทาง
        else:
            twist.linear.x = 0.25  # วิ่งไปข้างหน้า
    
    def line_of_sight(self):
        # ตรวจสอบว่าไม่มีสิ่งกีดขวางด้านหน้า ซ้าย และขวา
        return (self.front_distance > self.escape_threshold and
                self.left_distance > self.escape_threshold and
                self.right_distance > self.escape_threshold)

    
    def reached_goal(self):
        distance = math.sqrt((self.goal_x - self.position['x'])**2 + (self.goal_y - self.position['y'])**2)
        return distance < 0.1


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotBugAlgorithm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
