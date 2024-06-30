import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
import traceback
from example_interfaces.msg import Float32
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from aruco_interfaces.msg import ArucoMarkers

from geometry_msgs.msg import Twist
import sys
import time
import math





import requests
API_TOKEN = '7299667114:AAGlaT3-b5hAG0SrGdqynGxgwThRWAH_iI8'
GROUP_CHAT_ID = '1745570526'
def sendBot(message:str):
    response = requests.get(f'https://api.telegram.org/bot{API_TOKEN}/sendMessage?chat_id={GROUP_CHAT_ID}&text={message}')

import asyncio

class FollowArucoMarker(Node):
    def __init__(self):
        super().__init__('follow_aruco_marker')
        self.get_logger().info("Starting follow aruco marker behavior!")
        self.aruco_pose = None
        self.rpm_wheel = 0.0

        self.markers_found = []
        self.number_of_the_marker_we_are_going_to = None
        
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        self.subscription_markers = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.markers_callback,
            10)


        self.subscription_rpm = self.create_subscription(
            Float32,
            'left_wheel_rpm',
            self.rpm_callback,
            2)
        
       
        # self.max_distance = 0.7
        self.detect_marker = False
        self.rotation_span = 2.7
        self.waiting_span = 0.3
        self.min_distance = 0.95
        self.distance_to_marker = 0.0
        
        self.lin_max_speed = 0.17
        self.rot_max_speed = 0.013
        self.t0 = time.time()

        self.last_publish = time.time()
        self._main_cycle()

    def event(self, event):
        print(event)
    def emergency_shutdown(self):
            self.get_logger().warn("Following aruco marker emergency shutdown! Spamming a Twist of 0s!")
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            for i in range(4):
                self.cmd_pub.publish(twist)
                time.sleep(0.05)

    def markers_callback(self, msg:'ArucoMarkers'):
        #print(f'markers_callback')
        self.aruco_pose = msg

        if self.number_of_the_marker_we_are_going_to is None:
            self._find_marker()
        else:
            self._go_to_marker()
    def rpm_callback(self, msg):
        #print(f'rpm_callback')
        self.rpm_wheel = msg

        self._main_cycle()

    def _main_cycle(self):
        if self.detect_marker == False:
            self._main_action()
            #print(f'Обновление основного потока движения')
    def _main_action(self):
        rot_speed = 0.1
        lin_speed = 0.0
  
        
        twist = Twist()
        twist.linear.x = lin_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rot_speed
        self.publish(twist)
    def publish(self, twist:'Twist'):
        current_time = time.time()
        if self.last_publish < current_time - 0.4:
            self.cmd_pub.publish(twist)
            self.get_logger().info("lin_speed={:.2f}, rot_speed={:.2f}".format(
                twist.linear.x, twist.angular.z ))
            self.last_publish = current_time
        else:
            pass
            #print('Ожиданиее потока')

    def _go_to_marker(self):
        _uni_marker_id = self.aruco_pose.marker_ids[0]
        if self.number_of_the_marker_we_are_going_to is not None and (_uni_marker_id != self.number_of_the_marker_we_are_going_to):
            self._send_message_about_marker(f'Потерял маркер из виду. Был {self.number_of_the_marker_we_are_going_to}, а стал {_uni_marker_id}')
            self.detect_marker = False
            self.number_of_the_marker_we_are_going_to = None
            return
        
        self.number_of_the_marker_we_are_going_to = _uni_marker_id


        z = self.aruco_pose.poses[0].position.z
        x = self.aruco_pose.poses[0].position.x
        y = self.aruco_pose.poses[0].position.y

        print(f'Поворот -> {x}')
        print(f'Высота -> {y}')
        print(f'Растояние -> {z}')
        
        x2 = self.aruco_pose.poses[0].orientation.x
        y2 = self.aruco_pose.poses[0].orientation.y
        z2 = self.aruco_pose.poses[0].orientation.z
        w2 = self.aruco_pose.poses[0].orientation.w
        
        print(f'orientation -> {x2},{y2},{z2},{w2}')

        if z < 0.5:
            self.number_of_the_marker_we_are_going_to = None

            print(f'New marker -> {_uni_marker_id}')

            self._send_message_about_marker(_uni_marker_id)
            self.markers_found.append(_uni_marker_id)

            self.detect_marker = False
            self._go_back_from_marker()
            #self._main_cycle()
            return

        
        twist = Twist()
        twist.linear.x = 0.08      # стабильная 0.02
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -x          # стабильная  -x/5
        self.cmd_pub.publish(twist)
        self._main_cycle()
    def _go_back_from_marker(self):

        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
    def _find_marker(self):
        _uni_marker_id = self.aruco_pose.marker_ids[0]
        if _uni_marker_id in self.markers_found:
            print(f'marker return -> {_uni_marker_id}')
            #return
        

        self.detect_marker = True
        self._go_to_marker()
        

    def _send_message_about_marker(self, uni_marker_id:int):
        message = f'I found marker -> "{uni_marker_id}"'
        print(message)
        sendBot(message)
def main(args=None):
    rclpy.init(args=args)
    try:
        node = FollowArucoMarker()


        # qtimer = QTimer()
        # qtimer.timeout.connect(lambda: node._main_cycle)
        # qtimer.start(500)
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init Following aruco marker')
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
