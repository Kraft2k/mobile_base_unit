import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time
import traceback
import requests
import sys







API_TOKEN = '7299667114:AAGlaT3-b5hAG0SrGdqynGxgwThRWAH_iI8'
GROUP_CHAT_ID = '1745570526'
def sendBot(message: str):
    response = requests.get(f'https://api.telegram.org/bot{API_TOKEN}/sendMessage?chat_id={GROUP_CHAT_ID}&text={message}')

class DetectDoorClient(Node):
    def __init__(self):
        super().__init__('detect_door')
        self.get_logger().info("Starting detect door behavior!")
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
        
        self.laser_scan = None
        self.door_found = False
        self.min_door_width = 0.8  # минимальная ширина двери в метрах
        self.min_distance_to_wall = 0.1  # минимальное расстояние до стены в метрах
        self.safe_distance = 0.3  # безопасное расстояние до препятствий
        self.rotate_timer = None
        self.navigate_timer = None
        self.twist = Twist()
        self.exploring = False
        self.exploration_timer = None
        self.backing_off = False
        self.last_log_time = self.get_clock().now()

    def emergency_shutdown(self):
        self.get_logger().warn("Behavior detect and speech emergency shutdown! Spamming a Twist of 0s!")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        for i in range(4):
            self.cmd_pub.publish(twist)
            self.get_logger().warn("Spamming Twist of 0s!")
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def scan_callback(self, msg):
        self.laser_scan = msg
        if not self.backing_off:
            self.explore_room()

    def explore_room(self):
        if not self.laser_scan:
            return
        
        if not self.exploring:
            self.exploring = True
            self.exploration_timer = self.create_timer(1.0, self.explore_step)

    def explore_step(self):
        if not self.laser_scan:
            return

        ranges = self.laser_scan.ranges
        angle_increment = self.laser_scan.angle_increment
        
        # Проверка на наличие большого прохода перед роботом
        center_index = len(ranges) // 2
        front_distances = ranges[center_index - 10:center_index + 10]
        min_distance_ahead = min(front_distances)
        
        if str(min_distance_ahead) == 'inf':
            return


        if min_distance_ahead > self.min_door_width:
            self.door_found = True
            self.log_info(f"Found wide passage with distance {min_distance_ahead:.2f} meters, moving forward")
            self.move_forward()
        else:
            self.log_info("Wide passage not found, continuing exploration...")
            if min(ranges) < self.safe_distance:
                self.log_info("Obstacle detected, backing off...")
                self.back_off()
            else:
                self.rotate()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.1  # Устанавливаем скорость вперед (более плавное движение)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.exploring = False

    def back_off(self):
        self.backing_off = True
        twist = Twist()
        twist.linear.x = -0.1  # Движение назад (более плавное движение)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.create_timer(1.0, self.stop_back_off)

    def stop_back_off(self):
        self.backing_off = False
        self.twist.linear.x = 0.0
        self.cmd_pub.publish(self.twist)
        self.rotate()

    def rotate(self):
        if self.rotate_timer is None or not self.rotate_timer.is_ready():
            self.twist.angular.z = 0.2  # Регулируйте скорость поворота по необходимости (более плавное движение)
            self.cmd_pub.publish(self.twist)
            self.rotate_timer = self.create_timer(1.0, self.stop_rotation)

    def stop_rotation(self):
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        self.rotate_timer = None
        self.exploring = False

    def log_info(self, message: str):
        current_time = self.get_clock().now()
        if (current_time - self.last_log_time).nanoseconds > 1e9:  # Логировать не чаще одного раза в секунду
            self.get_logger().info(message)
            self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DetectDoorClient()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init detect door')
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
