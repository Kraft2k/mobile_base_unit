import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy
from geometry_msgs.msg import Twist
import csv
import os

class RouteController(Node):

    def __init__(self):
        super().__init__('nav_controller')
        self.get_logger().info("Navigation controller has been started")
        
        # Subscriptions
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10)
        
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        self.vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.route = []
        self.recording = False
        self.replaying = False
        self.route_file = 'route_with_velocities.csv'
        self.route_index = 0
        self.load_route()

    def gps_callback(self, msg):
        if self.recording:
            if hasattr(self, 'last_twist'):
                self.route.append((
                    msg.latitude, 
                    msg.longitude, 
                    self.last_twist.linear.x, 
                    self.last_twist.angular.z
                ))
                self.save_route()

    def vel_callback(self, msg):
        if self.recording:
            self.last_twist = msg

    def save_route(self):
        with open(self.route_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.route)

    def load_route(self):
        if os.path.exists(self.route_file):
            with open(self.route_file, newline='') as csvfile:
                reader = csv.reader(csvfile)
                self.route = [(float(lat), float(lon), float(lin_vel), float(ang_vel)) 
                              for lat, lon, lin_vel, ang_vel in reader]

    def navigate_to_next_point(self):
        if self.route_index < len(self.route):
            lat, lon, lin_vel, ang_vel = self.route[self.route_index]
            twist = Twist()
            twist.linear.x = lin_vel
            twist.angular.z = ang_vel
            self.publisher_.publish(twist)
            self.route_index += 1

    def start_recording(self):
        self.recording = True
        self.replaying = False  # Ensure replaying is stopped
        self.route = []  # Clear previous route
        self.get_logger().info("Started recording the route.")

    def stop_recording(self):
        self.recording = False
        self.save_route()
        self.get_logger().info("Stopped recording the route and saved it.")

    def start_replaying(self):
        self.replaying = True
        self.recording = False  # Ensure recording is stopped
        self.route_index = 0
        self.get_logger().info("Started replaying the route.")

    def stop_replaying(self):
        self.replaying = False
        self.get_logger().info("Stopped replaying the route.")

    def joy_callback(self, msg):
        if self.recording:
            if msg.buttons[3]:  # Square button
                self.stop_recording()
        elif self.replaying:
            if msg.buttons[0]:  # Cross button
                self.stop_replaying()
        else:
            if msg.buttons[1]:  # Circle button
                self.start_recording()
            elif msg.buttons[2]:  # Triangle button
                self.start_replaying()

def main(args=None):
    rclpy.init(args=args)
    nav_controller = RouteController()

    try:
        while rclpy.ok():
            rclpy.spin_once(nav_controller)
    except KeyboardInterrupt:
        pass
    finally:
        nav_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()