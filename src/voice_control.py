import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2
import tf_transformations
import traceback
from example_interfaces.msg import Float32
from example_interfaces.msg import String

# from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import Twist
import sys
import time
import math



class VoiceCommandControl(Node):
    def __init__(self):
        super().__init__('voice_command_control')
        self.get_logger().info("Starting voice command control behavior!")
        self.command = String
        self.rpm_wheel = 0.0
        
        self.subscription_markers = self.create_subscription(
            String,
            'command_publish',
            self.command_callback,
            10)

               
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)


        self.lin_max_speed = 0.17
        self.rot_max_speed = 0.03
        self.t0 = time.time()
        
    
        self.create_timer(0.1, self.main_tick)


    def emergency_shutdown(self):
            self.get_logger().warn("Voice command control emergency shutdown! Spamming a Twist of 0s!")
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

    def command_callback(self, msg):

        
        self.command = msg
        #self.get_logger().info('command is {0}'.format(str(self.command.data)))

    

    def main_tick(self):
        
        
        self.get_logger().info("Control command: {}".format(self.command.data))
        if self.command.data in ['вези', 'вперёд', 'перед']:
            lin_speed = self.lin_max_speed
            rot_speed = 0.0

    

        elif self.command.data == 'назад':
            lin_speed = -self.lin_max_speed
            rot_speed = 0.0

        elif self.command.data == 'левее':
            lin_speed = self.lin_max_speed
            rot_speed = self.rot_max_speed

        elif self.command.data == 'налево':
            lin_speed = 0.0
            rot_speed = self.rot_max_speed

        elif self.command.data == 'правее':
            lin_speed = self.lin_max_speed
            rot_speed = -self.rot_max_speed

        elif self.command.data == 'направо':
            lin_speed = 0.0
            rot_speed = -self.rot_max_speed

        elif (self.command.data == 'стоп'):
            lin_speed = 0.0
            rot_speed = 0.0
        
        else:
            lin_speed = 0.0
            rot_speed = 0.0

        

        

        #     print(str(self.aruco_pose.marker_ids[0]))
        #     self.distance_to_marker = self.aruco_pose.poses[0].position.z
        #     print(str(self.distance_to_marker))
        #     if self.distance_to_marker > self.min_distance:
        #         lin_speed = self.lin_max_speed
        #         rot_speed = 0.0
        #     else:
        #         lin_speed = 0.0
        #         rot_speed = 0.0
        #     self.detect_marker = True


        # else:
        #     current_time = time.time()
        #     print("self.aruco_pose is None")
        #     lin_speed = 0.0
        #     if (current_time - self.t0 <= self.rotation_span):
        #         rot_speed = -self.rot_max_speed
        #     elif (current_time - self.t0 <= self.rotation_span + self.waiting_span):
        #         rot_speed = 0.0
        #     else:
        #         self.t0 = time.time()
        #         rot_speed = 0.0
        #     lin_speed = 0.0
    
              
  
        self.get_logger().info("lin_speed={:.2f}, rot_speed={:.2f}".format(lin_speed, rot_speed))
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
        node = VoiceCommandControl()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init Voice command control')
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
