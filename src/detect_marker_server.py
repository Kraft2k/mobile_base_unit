#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from units_interfaces.action import DetectMarker
import time



class DetectMarkerServer(Node): 
    def __init__(self): 
        super().__init__("detect_marker_server")

        self.detect_marker_server = ActionServer(
            self,
            DetectMarker, 
            "detect_marker",
            execute_callback=self.execute_callback)
        self.get_logger().info("Detect marker server has been started")

    def run_speech(self, id_marker):
        
        self.get_logger().info(f'Hello, Liba_bot has been detected marker with number: {id_marker}')
        # Put here script of speech
        

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get request from goal
        id_marker = goal_handle.request.id_marker
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period


        # Execute the action
        
        self.run_speech(id_marker)
        counter = 0 
        for i in range(target_number):
            counter += 1
            time.sleep(period)
            self.get_logger().info(str(counter))
        

        # Once done, set goal final state
        goal_handle.succeed()

        # and send the result
        result =  DetectMarker.Result()
        result.reached_number = counter
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DetectMarkerServer() 
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == "__main__":
    main()