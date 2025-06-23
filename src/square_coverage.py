#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SquareCoverage:
    def __init__(self):
        rclpy.init_node('square_coverage')
        self.cmd_pub = rclpy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rclpy.Subscriber('/odom', Odometry, self.odom_cb)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.side_length = 2.0  # 2 метра
        self.swath_spacing = 0.5  # Шаг между линиями

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Получаем угол из кватерниона (упрощенно)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def move_straight(self, distance):
        # Движение вперед на заданное расстояние
        start_x, start_y = self.current_x, self.current_y
        while math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2) < distance:
            cmd = Twist()
            cmd.linear.x = 0.2  # 0.2 м/с
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        self.stop()

    def turn(self, angle):
        # Поворот на заданный угол (в радианах)
        target_yaw = self.current_yaw + angle
        while abs(self.current_yaw - target_yaw) > 0.1:
            cmd = Twist()
            cmd.angular.z = 0.3 if angle > 0 else -0.3
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        self.stop()

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.5)

    def run(self):
        # Простейший паттерн "змейка"
        for i in range(int(self.side_length / self.swath_spacing)):
            self.move_straight(self.side_length)
            if i % 2 == 0:
                self.turn(math.pi/2)
                self.move_straight(self.swath_spacing)
                self.turn(math.pi/2)
            else:
                self.turn(-math.pi/2)
                self.move_straight(self.swath_spacing)
                self.turn(-math.pi/2)

if __name__ == '__main__':
    node = SquareCoverage()
    rospy.sleep(1)  # Ждем инициализации
    node.run()