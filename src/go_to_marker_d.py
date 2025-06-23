import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from aruco_interfaces.msg import ArucoMarkers
import tf_transformations as tf

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
        self.rot_max_speed = 0.013  # Maximum angular speed (rad/s)

        self.get_logger().warn("Marker follower has been started")
        
    def marker_callback(self, msg):
        if len(msg.marker_ids) > 0:
            marker_pose = msg.poses[0]  # Предполагаем, что обнаружен один маркер
            position = marker_pose.position
            orientation = marker_pose.orientation
            
            # Преобразование кватерниона в углы Эйлера
            roll, pitch, yaw = tf.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

            # Рассчитываем расстояние до маркера
            distance = (position.x**2 + position.y**2 + position.z**2) ** 0.5

            # Рассчитываем угол отклонения от перпендикуляра
            move_angle = -yaw  # Учитываем, что yaw должен быть близок к 0 для перпендикуляра
            
            # Ограничение скоростей
            move_distance = distance - self.target_distance
            move_distance = max(min(move_distance, self.lin_max_speed), -self.lin_max_speed)
            move_angle = max(min(move_angle, self.rot_max_speed), -self.rot_max_speed)
            
            # Команда для перемещения ровера
            cmd_msg = Twist()
            if abs(move_angle) > 0.05:  # Порог для углового движения, чтобы ровер был строго перпендикулярен
                cmd_msg.angular.z = move_angle  # Корректируем ориентацию ровера
            else:
                cmd_msg.linear.x = move_distance  # Двигаемся вперед/назад
            
            if move_distance < 0.05 and abs(move_angle) < 0.05:  # Если в пределах целевой дистанции и перпендикулярен, остановка
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