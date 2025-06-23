import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist
import cv2
import time

class SegNetMovingNode(Node):
    def __init__(self):
        super().__init__('segnet_moving')
        self.get_logger().info("Starting SegNet moving behavior!")
        self.subscription = self.create_subscription(
            Image,
            '/segnet/class_mask',
            self.class_mask_callback,
            10)
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.lin_max_speed = 0.09
        self.lin_reverse_speed = -0.05  # Reverse speed
        self.rot_max_speed = 0.02  # Adjust rotation speed as needed
        self.floor_mask = 2
        self.table_mask = 7
        self.acceptable_masks = [self.floor_mask, self.table_mask]
        self.t0 = time.time()

        # Parameters for noise filtering
        self.kernel = np.ones((5, 5), np.uint8)  # Kernel for morphological operations
        self.mask_history = deque(maxlen=5)  # History of masks for temporal smoothing

        # Variables for state management
        self.state = 'forward'  # Possible states: forward, waiting, turning_left, turning_right
        self.obstacle_detected_time = None
        self.reverse_start_time = None
        self.waiting_start_time = None
        self.turning_start_time = None
        self.turn_phase = None  # Initialize turn phase

        # Coefficient to reduce ROI width
        self.roi_width_coefficient = 0.1  # Reduce ROI width to 10% of image width

        # Threshold for obstacle detection
        self.obstacle_threshold = 1  # Decreased threshold due to reduced ROI

    def emergency_shutdown(self):
        self.get_logger().warn("SegNet moving emergency shutdown!")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.05)

    def class_mask_callback(self, msg):
        try:
            class_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            height, width = class_mask.shape

            # Apply morphological operations to remove noise
            filtered_mask = cv2.morphologyEx(class_mask, cv2.MORPH_OPEN, self.kernel)
            filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, self.kernel)

            # Add mask to history and compute average
            self.mask_history.append(filtered_mask)
            if len(self.mask_history) == self.mask_history.maxlen:
                averaged_mask = np.mean(self.mask_history, axis=0).astype(np.uint8)
            else:
                averaged_mask = filtered_mask

            # Continue processing with the averaged mask
            class_mask = averaged_mask

            # Offset due to camera being shifted 10 cm to the left
            rover_width_cm = 40  # Width of the rover in cm
            camera_offset_cm = 10  # Camera offset in cm
            offset_fraction = camera_offset_cm / rover_width_cm
            offset_pixels = int(offset_fraction * width)

            # Define ROI height (lower part of the image)
            roi_height = int(0.2 * height)
            roi_start_y = height - roi_height
            roi_end_y = height

            # Define ROI width with coefficient
            roi_width = int(self.roi_width_coefficient * width)

            # Adjust ROI considering camera offset
            # Left ROI
            left_roi_start_x = max(0, int(width * 0.25) + offset_pixels)
            left_roi_end_x = left_roi_start_x + roi_width
            left_roi_end_x = min(left_roi_end_x, width)
            left_roi = class_mask[roi_start_y:roi_end_y, left_roi_start_x:left_roi_end_x]

            # Center ROI
            center_roi_start_x = int((width - roi_width) / 2) + offset_pixels
            center_roi_start_x = min(center_roi_start_x, width - roi_width)
            center_roi_end_x = center_roi_start_x + roi_width
            center_roi_end_x = min(center_roi_end_x, width)
            center_roi = class_mask[roi_start_y:roi_end_y, center_roi_start_x:center_roi_end_x]

            # Right ROI
            right_roi_start_x = max(0, int(width * 0.75) + offset_pixels - roi_width)
            right_roi_end_x = right_roi_start_x + roi_width
            right_roi_end_x = min(right_roi_end_x, width)
            right_roi = class_mask[roi_start_y:roi_end_y, right_roi_start_x:right_roi_end_x]

            # Count obstacle pixels in each ROI
            left_obstacle_pixels = np.count_nonzero(~np.isin(left_roi, self.acceptable_masks))
            center_obstacle_pixels = np.count_nonzero(~np.isin(center_roi, self.acceptable_masks))
            right_obstacle_pixels = np.count_nonzero(~np.isin(right_roi, self.acceptable_masks))

            self.get_logger().info(f'State: {self.state}')
            self.get_logger().info(f'Obstacle pixels - Left: {left_obstacle_pixels}, Center: {center_obstacle_pixels}, Right: {right_obstacle_pixels}')

            current_time = time.time()

            # Ensure time variables are initialized
            if self.reverse_start_time is None:
                self.reverse_start_time = current_time
            if self.waiting_start_time is None:
                self.waiting_start_time = current_time
            if self.turning_start_time is None:
                self.turning_start_time = current_time

            if left_obstacle_pixels <= self.obstacle_threshold and center_obstacle_pixels <= self.obstacle_threshold and right_obstacle_pixels <= self.obstacle_threshold:
                self.state = 'forward'

            if self.state == 'forward':
                if left_obstacle_pixels > self.obstacle_threshold or center_obstacle_pixels > self.obstacle_threshold or right_obstacle_pixels > self.obstacle_threshold:
                    # Obstacle detected
                    self.get_logger().info('Obstacle detected ahead. Stopping and waiting.')
                    self.stop_rover()
                    self.reverse_start_time = current_time
                    self.state = 'waiting'
                else:
                    # No obstacles, continue moving forward
                    self.move_forward()
            elif self.state == 'waiting':
                if current_time - self.waiting_start_time < 2.0:
                    # Wait for 2 seconds
                    pass
                else:
                    # Decide which direction to turn
                    if left_obstacle_pixels < right_obstacle_pixels:
                        self.get_logger().info('Navigating around obstacle to the left.')
                        self.turning_start_time = current_time
                        self.state = 'turning_left'
                    elif right_obstacle_pixels < left_obstacle_pixels:
                        self.get_logger().info('Navigating around obstacle to the right.')
                        self.turning_start_time = current_time
                        self.state = 'turning_right'
                    else:
                        self.get_logger().info('Navigating around obstacle to the right.')
                        self.turning_start_time = current_time
                        self.state = 'turning_right'
                        # # Obstacles on both sides
                        # self.get_logger().info('Obstacles on both sides. Remaining in place.')
                        # self.stop_rover()
            elif self.state == 'turning_left':
                if current_time - self.turning_start_time < 1.0:
                    # Turning left with stops
                    self.turn_with_stops(direction='left')
                else:
                    # Finished turning, return to moving forward
                    self.state = 'forward'
                    self.turn_phase = None  # Reset turn phase
                    self.turning_start_time = None
            elif self.state == 'turning_right':
                if current_time - self.turning_start_time < 1.0:
                    # Turning right with stops
                    self.turn_with_stops(direction='right')
                else:
                    # Finished turning, return to moving forward
                    self.state = 'forward'
                    self.turn_phase = None  # Reset turn phase
                    self.turning_start_time = None
            else:
                # Unknown state, stopping
                self.stop_rover()
                self.state = 'forward'
        except Exception as e:
            self.get_logger().error('Error processing class_mask: %s' % str(e))

    def stop_rover(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.lin_max_speed
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def move_backward(self):
        twist = Twist()
        twist.linear.x = self.lin_reverse_speed
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def turn_with_stops(self, direction):
        # Turn for a small interval, then stop and assess the surroundings
        turn_duration = 0.2  # Duration of turning before stopping
        stop_duration = 0.1   # Duration of stopping to assess surroundings
        current_time = time.time()

        if not hasattr(self, 'turn_phase') or self.turn_phase is None:
            self.turn_phase = 'turning'
            self.phase_start_time = current_time

        if self.turn_phase == 'turning':
            if current_time - self.phase_start_time < turn_duration:
                # Continue turning
                twist = Twist()
                twist.linear.x = 0.0
                if direction == 'left':
                    twist.angular.z = self.rot_max_speed
                else:
                    twist.angular.z = -self.rot_max_speed
                self.cmd_pub.publish(twist)
            else:
                # Switch to stopping phase
                self.turn_phase = 'stopping'
                self.phase_start_time = current_time
                self.stop_rover()
        elif self.turn_phase == 'stopping':
            if current_time - self.phase_start_time < stop_duration:
                # Stop and assess surroundings
                self.stop_rover()
            else:
                # Return to turning phase
                self.turn_phase = 'turning'
                self.phase_start_time = current_time
        else:
            # If something went wrong, stop
            self.stop_rover()
            self.turn_phase = 'turning'
            self.phase_start_time = current_time

    def turn_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.rot_max_speed  # Positive value for turning left
        self.cmd_pub.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.rot_max_speed  # Negative value for turning right
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rover_controller = SegNetMovingNode()
    rclpy.spin(rover_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()