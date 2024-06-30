import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time
import traceback
import requests
import sys





import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QPainter, QColor, QPen
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MapVisualizer(QMainWindow):
    def __init__(self, map_data, parent=None):
        super().__init__(parent)
        self.map_data = map_data
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Room Map')
        self.setGeometry(100, 100, 800, 800)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.setCentralWidget(self.canvas)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(1000)

    def update_map(self):
        self.figure.clear()
        ax = self.figure.add_subplot(111)

        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)

        for point in self.map_data:
            ax.plot(point[0], point[1], 'ro' if point[2] == 1 else 'bo')

        ax.plot(self.map_data[-1][0], self.map_data[-1][1], 'go')  # Робот
        self.canvas.draw()

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
        self.map_data = []
        self.robot_position = [0, 0]
        self.robot_orientation = 0
        self.visualizer = None
        self.create_visualizer()

        self.min_door_width = 0.5
        self.min_distance_to_wall = 0.1
        self.safe_distance = 0.3
        self.rotate_timer = None
        self.navigate_timer = None
        self.twist = Twist()
        self.exploring = False
        self.exploration_timer = None
        self.backing_off = False

    def create_visualizer(self):
        app = QApplication([])
        self.visualizer = MapVisualizer(self.map_data)
        self.visualizer.show()
        threading.Thread(target=app.exec_).start()

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
            self.update_map()
            self.explore_room()

    def update_map(self):
        ranges = self.laser_scan.ranges
        angle_increment = self.laser_scan.angle_increment

        for i, distance in enumerate(ranges):
            if distance > self.laser_scan.range_min and distance < self.laser_scan.range_max:
                angle = self.robot_orientation + (i * angle_increment)
                x = self.robot_position[0] + distance * math.cos(angle)
                y = self.robot_position[1] + distance * math.sin(angle)
                self.map_data.append((x, y, 1))

        self.map_data.append((self.robot_position[0], self.robot_position[1], 0))  # Робот

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
        center_index = len(ranges) // 2
        front_distances = ranges[center_index - 10:center_index + 10]
        min_distance_ahead = min(front_distances)

        if min_distance_ahead > self.min_door_width:
            self.door_found = True
            self.get_logger().info(f"Found wide passage with distance {min_distance_ahead:.2f} meters, moving forward")
            self.move_forward()
        else:
            self.get_logger().info("Wide passage not found, continuing exploration...")
            if min(ranges) < self.safe_distance:
                self.get_logger().info("Obstacle detected, backing off...")
                self.back_off()
            else:
                self.rotate()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.update_robot_position(0.1, 0.0)
        self.exploring = False

    def back_off(self):
        self.backing_off = True
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.update_robot_position(-0.1, 0.0)
        self.create_timer(1.0, self.stop_back_off)

    def stop_back_off(self):
        self.backing_off = False
        self.twist.linear.x = 0.0
        self.cmd_pub.publish(self.twist)
        self.rotate()

    def rotate(self):
        if self.rotate_timer is None or not self.rotate_timer.is_ready():
            self.twist.angular.z = 0.2
            self.cmd_pub.publish(self.twist)
            self.update_robot_position(0.0, 0.2)
            self.rotate_timer = self.create_timer(1.0, self.stop_rotation)

    def stop_rotation(self):
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        self.rotate_timer = None
        self.exploring = False

    def update_robot_position(self, linear_distance, angular_distance):
        self.robot_position[0] += linear_distance * math.cos(self.robot_orientation)
        self.robot_position[1] += linear_distance * math.sin(self.robot_orientation)
        self.robot_orientation += angular_distance

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
