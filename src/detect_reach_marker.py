import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from units_interfaces.action import UpFrontServo

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



class DetectReachMarker(Node):
    def __init__(self):
        super().__init__('detect_and_reach_marker')
        self.get_logger().info("Starting detect and reach marker behavior!")
        self.aruco_pose = None
        self.rpm_wheel = 0.0
        
        self.subscription_markers = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.markers_callback,
            10)
       
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        self.up_front_servo_client_ = ActionClient(self, UpFrontServo, "up_front_servo")

        # self.max_distance = 0.7
        self.detected_marker = False
        self.reached_marker = False
        self.rotation_span = 2.7
        self.waiting_span = 0.3
        self.reached_distance = 0.95
        self.distance_to_marker = 0.0
        
        self.lin_max_speed = 0.17
        self.rot_max_speed = 0.013
        self.t0 = time.time()
        #time.sleep(1)
    
        self.create_timer(0.01, self.main_tick)


    def emergency_shutdown(self):
            self.get_logger().warn("Behavior detect and reach marker emergency shutdown! Spamming a Twist of 0s!")
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

    def send_goal(self, target_number, period):
        # Wait for the server
        self.up_front_servo_client_.wait_for_server()

        # Create a goal
        goal = UpFrontServo.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self.get_logger().info("Sending goal")
        self.up_front_servo_client_. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: " + str(result.reached_number))

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
            if self.distance_to_marker > self.reached_distance:
                lin_speed = self.lin_max_speed
                rot_speed = 0.0
            else:
                lin_speed = 0.0
                rot_speed = 0.0
                if not self.reached_marker:
                    self.send_goal(19, 1.0)
                    self.reached_marker = True
                    self.aruco_pose = None
            self.detected_marker = True


        else:
            current_time = time.time()
            print("self.aruco_pose is None")
            lin_speed = 0.0
            if (current_time - self.t0 <= self.rotation_span):
                rot_speed = -self.rot_max_speed
            elif (current_time - self.t0 <= self.rotation_span + self.waiting_span):
                rot_speed = 0.0
            else:
                self.t0 = time.time()
                rot_speed = 0.0
            lin_speed = 0.0
    
              
  
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
        node = DetectReachMarker()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init detect and reach marker')
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
