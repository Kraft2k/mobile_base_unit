import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from aruco_interfaces.msg import ArucoMarkers
import math
import tf_transformations
import tf2_ros
import tf2_geometry_msgs
import tf2_py
from rclpy.duration import Duration

class MarkerFollower(Node):
    def __init__(self):
        super().__init__('marker_follower')
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.marker_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_distance = 0.5  # Целевое расстояние до маркера (м)
        self.lin_max_speed = 0.10   # Максимальная линейная скорость (м/с)
        self.rot_max_speed = 0.07   # Максимальная угловая скорость (рад/с)
        self.get_logger().info("Marker follower has been started")

        # Настройка tf2 буфера и слушателя
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def marker_callback(self, msg):
        if len(msg.marker_ids) > 0:
            marker_pose = msg.poses[0]

            # Создание сообщения PoseStamped для преобразования
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = msg.header.frame_id  # 'camera_color_optical_frame'
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()  # Устанавливаем время на 0
            pose_stamped.pose = marker_pose

            try:
                # Преобразование позы в систему координат робота
                transformed_pose = self.tf_buffer.transform(
                    pose_stamped,
                    'base_link',  # Замените на фрейм вашего робота
                    timeout=Duration(seconds=0.5)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Transform error: {e}")
                return

            position = transformed_pose.pose.position
            orientation = transformed_pose.pose.orientation

            # Вычисление ошибки по расстоянию
            distance = math.hypot(position.x, position.y)
            distance_error = distance - self.target_distance

            # Вычисление угла до маркера относительно робота
            angle_error = math.atan2(position.y, position.x)
            # Нормализация угловой ошибки в диапазон [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            cmd_msg = Twist()

        

            # Уменьшить коэффициент углового П-контроллера
            k_ang = 1.0  # было 2.0, уменьшение для плавного поворота

            # Увеличить коэффициент линейного П-контроллера
            k_lin = 1.0  # было 0.7, увеличение для большей уверенности движения вперед

            

            # Зоны нечувствительности
            distance_deadband = 0.05  # 5 см
            angle_deadband = math.radians(2)  # 2 градуса

            # Добавить условие для линейного движения при наличии угловой ошибки
            if abs(angle_error) < math.radians(20):  # Позволить линейное движение при угловой ошибке менее 10 градусов
                cmd_msg.linear.x = max(min(k_lin * distance_error, max_linear_speed), -max_linear_speed)
            else:
                cmd_msg.linear.x = 0.0


           
            # Вычисление угловой скорости
            cmd_msg.angular.z = max(min(k_ang * angle_error, self.rot_max_speed), -self.rot_max_speed)

            # Применение зон нечувствительности
            if abs(distance_error) < distance_deadband:
                cmd_msg.linear.x = 0.0
            if abs(angle_error) < angle_deadband:
                cmd_msg.angular.z = 0.0

            # Логирование управляющих воздействий
            self.get_logger().info(
                f"Distance error: {distance_error:.2f} m, Angle error: {math.degrees(angle_error):.2f} degrees"
            )
            self.get_logger().info(
                f"Commanding linear.x: {cmd_msg.linear.x:.2f} m/s, angular.z: {cmd_msg.angular.z:.2f} rad/s"
            )
            self.get_logger().info(
                f"k_ang * angle_error: {k_ang * angle_error:.2f}, rot_max_speed: {self.rot_max_speed}"
            )

            # Остановка при достижении цели
            if cmd_msg.linear.x == 0.0 and cmd_msg.angular.z == 0.0:
                self.get_logger().info("Target position and orientation achieved.")

            # Публикация команды
            self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    marker_follower = MarkerFollower()
    rclpy.spin(marker_follower)
    marker_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
