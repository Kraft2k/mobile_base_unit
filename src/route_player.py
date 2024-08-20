import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import os
import time

class RoutePlayer(Node):

    def __init__(self):
        super().__init__('route_player')
        self.get_logger().info("Route player has been started")
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.route_file = 'route_with_velocities.csv'
        self.route = []
        self.route_index = 0
        self.load_route()

    def load_route(self):
        if os.path.exists(self.route_file):
            with open(self.route_file, newline='') as csvfile:
                reader = csv.reader(csvfile)
                self.route = [(float(lin_vel), float(ang_vel)) 
                              for lin_vel, ang_vel in reader]
                self.get_logger().info(f"Route loaded: {self.route}")

    def navigate_to_next_point(self):
        if self.route_index < len(self.route):
            lin_vel, ang_vel = self.route[self.route_index]
            twist = Twist()
            twist.linear.x = lin_vel
            twist.angular.z = ang_vel
            self.publisher_.publish(twist)
            self.get_logger().info(f"Published Twist: lin_vel={lin_vel}, ang_vel={ang_vel}")
            self.route_index += 1

    def timer_callback(self):
        self.navigate_to_next_point()
        if self.route_index >= len(self.route):
            self.get_logger().info("Finished replaying the route.")
            self.destroy_node()
            rclpy.shutdown()

    def start_replaying(self):
        self.get_logger().info("Started replaying the route.")
        self.create_timer(0.05, self.timer_callback)

def main(args=None):
    rclpy.init(args=args)
    route_player = RoutePlayer()

    try:
        route_player.start_replaying()
        rclpy.spin(route_player)
    except KeyboardInterrupt:
        pass
    finally:
        route_player.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()