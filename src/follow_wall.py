import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.scan_data = None
        self.distance_to_wall = 1.0  # Set your desired distance to the wall here
        self.linear_speed = 0.2  # Set your desired linear speed here
        self.angular_speed = 0.5  # Set your desired angular speed here

    def scan_callback(self, msg):
        self.scan_data = msg

    def follow_wall(self):
        if self.scan_data is not None:
            # Find the index corresponding to the forward direction
            forward_index = len(self.scan_data.ranges) // 2
            forward_distance = self.scan_data.ranges[forward_index]

            # Find the index corresponding to the right side direction
            right_index = 0
            right_distance = self.scan_data.ranges[right_index]

            # Adjust the right index based on your robot's orientation and lidar mounting
            # Ensure that the right_index corresponds to the direction perpendicular to the robot's forward direction

            error = right_distance - self.distance_to_wall

            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = -self.angular_speed * error  # Negative sign to follow the right wall

            self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    wall_follower_node = WallFollower()

    try:
        while rclpy.ok():
            wall_follower_node.follow_wall()
            rclpy.spin_once(wall_follower_node)

    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.shutdown()

if name == '__main__':
    main()