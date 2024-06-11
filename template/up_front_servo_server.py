#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from units_interfaces.action import UpFrontServo
import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class UpFrontServoServer(Node): 
    def __init__(self): 
        super().__init__("up_front_servo_server")

        i2c = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        pca = PCA9685(i2c)
        pca.frequency = 50

        self.front_servo = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2600)
        self.front_servo.fraction = 0.6

        self.up_front_servo_server_  = ActionServer(
            self,
            UpFrontServo, 
            "up_front_servo",
            execute_callback=self.execute_callback)
        self.get_logger().info("Action server has been started")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing up front servo")
        self.front_servo.fraction = 0.2
        counter = 0 
        for i in range(target_number):
            counter += 1
            time.sleep(period)
            self.get_logger().info(str(counter))
        self.front_servo.fraction = 0.6

        # Once done, set goal fianal state
        goal_handle.succeed()

        # and send the resul
        result =  UpFrontServo.Result()
        result.reached_number = counter
        return result

def main(args=None):
    rclpy.init(args=args)
    node = UpFrontServoServer() 
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == "__main__":
    main()