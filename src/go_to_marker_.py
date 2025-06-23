import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from aruco_interfaces.msg import ArucoMarkers
import tf_transformations as tf
import math

class MarkerFollower(Node):

    def __init__(self):
        super().__init__('marker_follower')
        
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.marker_callback,
            10)
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.target_distance = 0.8  # Target distance from marker (in meters)
        self.lin_max_speed = 0.17   # Maximum linear speed (m/s)
        self.rot_max_speed = 0.50  # Maximum angular speed (rad/s)

        self.get_logger().warn("Marker follower has been started")
        
    def marker_callback(self, msg):
        if len(msg.marker_ids) > 0:
            marker_pose = msg.poses[0]  # Предполагаем, что обнаружен один маркер
            position = marker_pose.position
            orientation = marker_pose.orientation
            
            # Преобразование кватерниона в углы Эйлера
            roll, pitch, yaw = tf.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

            # Положение маркера в координатах камеры
            marker_x = position.x
            marker_y = position.y
            marker_z = position.z

            # Рассчитываем расстояние до маркера
            distance = (marker_x**2 + marker_y**2 + marker_z**2) ** 0.5

            # Поворот для перпендикулярности к маркеру
            target_yaw = math.atan2(marker_y, marker_z)  # Угол между осью Z камеры и положением маркера в плоскости XZ
            
            # Инициализируем переменную move_distance
            move_distance = 0.0
            
            # Команда для перемещения ровера
            cmd_msg = Twist()
            
            # Рассчитываем движение вперед и угловую скорость одновременно
            move_distance = marker_z - self.target_distance
            cmd_msg.linear.x = max(min(move_distance, self.lin_max_speed), -self.lin_max_speed)
            cmd_msg.angular.z = max(min(target_yaw, self.rot_max_speed), -self.rot_max_speed)
            
            # Если достигли целевого расстояния и правильного угла, останавливаемся
            if abs(move_distance) < 0.05 and abs(target_yaw) < 0.05:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0

            self.get_logger().info("lin_speed={:.2f}, rot_speed={:.2f}".format(cmd_msg.linear.x, cmd_msg.angular.z))

            self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    marker_follower = MarkerFollower()

    rclpy.spin(marker_follower)

    marker_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()