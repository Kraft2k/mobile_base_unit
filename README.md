


colcon build --packages-select mobile_base_unit --symlink-install


ros2 run realsense2_camera realsense2_camera_node

ros2 run ros2_aruco aruco_node

ros2 run mobile_base_unit up_front_servo_server

ros2 run mobile_base_unit mobile_base





ros2 launch barra_description joystick.launch.py

ros2 launch barra_description rplidar.launch.py

ros2 run mobile_base_unit mobile_base




ros2 run mobile_base_unit detect_marker_server 

ros2 run mobile_base_unit detect_marker_client





