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



class FollowArucoMarker(Node):
    def __init__(self):
        super().__init__('follow_aruco_marker')
        self.get_logger().info("Starting follow aruco marker behavior!")
        self.aruco_pose = None
        self.rpm_wheel = 0.0
        
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
        
       
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        # self.max_distance = 0.7
        self.detect_marker = False
        self.rotation_span = 2.7
        self.waiting_span = 0.3
        self.min_distance = 0.95
        self.distance_to_marker = 0.0
        
        self.lin_max_speed = 0.17
        self.rot_max_speed = 0.013
        self.t0 = time.time()
        #time.sleep(1)
    
        self.create_timer(0.01, self.main_tick)


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

    def markers_callback(self, msg):

        # self.get_logger().info('Distance: {0}'.format(str(msg.data)))
        self.aruco_pose = msg

    def rpm_callback(self, msg):

        self.rpm_wheel = msg

        
    

    def main_tick(self, verbose=False):
        

        if self.aruco_pose is not None:
            print(str(self.aruco_pose.marker_ids[0]))
            print(str(self.aruco_pose))
            self.distance_to_marker = self.aruco_pose.poses[0].position.z
            print(str(self.distance_to_marker))
            if self.distance_to_marker > self.min_distance:
                lin_speed = self.lin_max_speed
                rot_speed = 0.0
            else:
                lin_speed = 0.0
                rot_speed = 0.0
            self.detect_marker = True


        else:
            current_time = time.time()
            print("self.aruco_pose is None")
            lin_speed = 0.0
            rot_speed = -self.rot_max_speed
            #if (current_time - self.t0 <= self.rotation_span):
            #    rot_speed = -self.rot_max_speed
            #elif (current_time - self.t0 <= self.rotation_span + self.waiting_span):
            #    rot_speed = 0.0
            #else:
            #    self.t0 = time.time()
            #    rot_speed = 0.0
            #lin_speed = 0.0
    
              
  
        self.get_logger().info("lin_speed={:.2f}, rot_speed={:.2f}".format(
            lin_speed, rot_speed))
        twist = Twist()
        twist.linear.x = lin_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rot_speed
        self.cmd_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    try:
        node = FollowArucoMarker()
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
