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



class Go(Node):
    def __init__(self):
        super().__init__('go')
        self.get_logger().info("Starting Go behavior!")
        self.command = String
        self.rpm_wheel = 0.0
        
           
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

    

    def main_tick(self):
        
        
 
        
        # left
        lin_speed = self.lin_max_speed
        rot_speed = self.rot_max_speed

        

        # # right
        # lin_speed = self.lin_max_speed
        # rot_speed = -self.rot_max_speed

        
        
  
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
        node = Go()
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