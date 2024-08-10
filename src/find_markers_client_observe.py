import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from units_interfaces.action import DetectMarker

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
import threading
import requests

from typing import Dict


API_TOKEN = '7299667114:AAGlaT3-b5hAG0SrGdqynGxgwThRWAH_iI8'
GROUP_CHAT_ID = '1745570526'
def sendBot(message:str):
    response = requests.get(f'https://api.telegram.org/bot{API_TOKEN}/sendMessage?chat_id={GROUP_CHAT_ID}&text={message}')


class MarkerObject():
    def __init__(self):
        self.is_sending_on_server = False
        self.len_road_to_marker = 0.0
        

class FindMarkersClient(Node):
    def __init__(self):
        super().__init__('find_markers_client')
        self.get_logger().info("Starting find markers client!")
        self.aruco_pose = None
        self.rpm_wheel = 0.0
        
        self.subscription_markers = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.markers_callback,
            10)
       
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        self.detect_marker_client_ = ActionClient(self, DetectMarker, "detect_marker")

        # self.max_distance = 0.7
        self.markers_found: Dict[int: MarkerObject] = {}
        self.markers_need_found = [2,1]
        self.is_on_order_find_markers = True
        self.number_of_the_marker_we_are_going_to = None
        self.detect_marker = False
        self.reached_marker = False
        self.rotation_span = 2.7
        self.waiting_span = 0.3
        self.reached_distance = 0.95
        self.distance_to_marker = 0.0
        
        self.lin_max_speed = 0.17
        self.rot_max_speed = 0.013
        self.t0 = time.time()
        #time.sleep(1)


        self.last_publish = time.time()
        self.create_timer(0.1, self._main_cycle)


    def emergency_shutdown(self):
            self.get_logger().warn("Find markers client emergency shutdown!")
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

    def send_goal(self, id_marker, target_number, period):
        # Wait for the server
        self.detect_marker_client_.wait_for_server()

        # Create a goal
        goal = DetectMarker.Goal()
        goal.id_marker = id_marker
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self.get_logger().info("Sending goal")
        self.detect_marker_client_. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: " + str(result.reached_number))
        self.get_logger().info('Получен сигнал от сервера')

        _uni_marker_id = self.aruco_pose.marker_ids[0]
        self._send_message_about_marker(_uni_marker_id)
        
        if _uni_marker_id in self.markers_need_found:
            self.markers_need_found.remove(_uni_marker_id)
            
            if self.markers_need_found == []:
                message = 'All markers found'
                print(message)
                sendBot(message)
                sys.exit()

        
        print(f'Completion in -> {self.markers_found[_uni_marker_id].len_road_to_marker}')
        def _():
            self.detect_marker = False
            print('Completing the backward path')
        timer = threading.Timer(self.markers_found[_uni_marker_id].len_road_to_marker*1.5, _)
        timer.start()

        self._go_back_from_marker()


    def markers_callback(self, msg:'ArucoMarkers'):
        #print(f'markers_callback')
        self.aruco_pose = msg

        if self.number_of_the_marker_we_are_going_to is None:
            self._find_marker()
        else:
            self._go_to_marker()
    def _main_cycle(self):
        if self.detect_marker == False:
            self._main_action()
            #print(f'Обновление основного потока движения')

    def _main_action(self):
        lin_speed = 0.0
        rot_speed = 0.1
     
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
            #print('Waitinf ror thread')

    def _go_to_marker(self):
        _uni_marker_id = self.aruco_pose.marker_ids[0]
        if self.number_of_the_marker_we_are_going_to is not None and (_uni_marker_id != self.number_of_the_marker_we_are_going_to):
            self._send_message_about_marker(f'The marker is lost. Was this: {self.number_of_the_marker_we_are_going_to}, Become this: {_uni_marker_id}')
            self.detect_marker = False
            self.number_of_the_marker_we_are_going_to = None
            return

        if self.markers_found[_uni_marker_id].is_sending_on_server is True:
            print('The marker id detected and its number is sent to the server...')
            return


        self.number_of_the_marker_we_are_going_to = _uni_marker_id


        z = self.aruco_pose.poses[0].position.z
        x = self.aruco_pose.poses[0].position.x
        y = self.aruco_pose.poses[0].position.y


        if z > self.markers_found[_uni_marker_id].len_road_to_marker:
            self.markers_found[_uni_marker_id].len_road_to_marker = z
        # print(f'Поворот -> {x}')
        # print(f'Down -> {y}')
        print(f'Distance -> {z}')
        
        x2 = self.aruco_pose.poses[0].orientation.x
        y2 = self.aruco_pose.poses[0].orientation.y
        z2 = self.aruco_pose.poses[0].orientation.z
        w2 = self.aruco_pose.poses[0].orientation.w
        
        # print(f'orientation -> {x2},{y2},{z2},{w2}')

        if z < 0.95:
            self.number_of_the_marker_we_are_going_to = None

            print(f'New marker -> {_uni_marker_id}')
            self.markers_found[_uni_marker_id].is_sending_on_server = True
            self.send_goal(self.aruco_pose.marker_ids[0], 9, 1.0)

            #self._main_cycle()
            return

        
        twist = Twist()
        twist.linear.x = 0.02      # stable 0.02
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -x/5          # stable  -x/5
        self.cmd_pub.publish(twist)
        self._main_cycle()
    def _go_back_from_marker(self):

        while self.detect_marker is True:
            time.sleep(0.1)

            twist = Twist()
            twist.linear.x = -0.1
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
        
    def _find_marker(self):
        _uni_marker_id = self.aruco_pose.marker_ids[0]
        if _uni_marker_id in self.markers_found:
            if self.markers_found[_uni_marker_id].is_sending_on_server is True:
                print(f'marker return -> {_uni_marker_id}')
                return
        else:
            if self.is_on_order_find_markers is False:
                if _uni_marker_id not in self.markers_need_found:
                    print('Detect unnecessary marker')
                    return
            else:
                if not _uni_marker_id == self.markers_need_found[0]:
                    print('Detect unnecessary marker')
                    return
        
        self.markers_found[_uni_marker_id] = MarkerObject()
        

        self.detect_marker = True
        self._go_to_marker()
        

    def _send_message_about_marker(self, uni_marker_id:int):
        message = f'I found marker -> "{uni_marker_id}"'
        print(message)
        sendBot(message)
    



def main(args=None):
    rclpy.init(args=args)
    try:
        node = FindMarkersClient()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init find markers client')
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
