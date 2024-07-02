import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('mobile_base_unit'),
        'config',
        'params.yaml'
    )


    
    realsense_camera_node = Node(
                package='realsense2_camera',
                name='realsense2_camera_node',
                executable='realsense2_camera_node',
                output='screen'
         )

    aruco_node = Node(
                package='ros2_aruco',
                name='aruco_node',
                executable='aruco_node',
                output='screen'
         )

    # rplidar_node = Node(
    #             package='rplidar_ros',
    #             executable='rplidar_composition',
    #             output='screen',
    #             parameters=[{

    #                 'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    #                 'frame_id': 'laser_frame',
    #                 'angle_compensate': True,
    #                 'scan_mode': 'Standard'
    #             }]
    #      )

    mobile_base_node = Node(
                package='mobile_base_unit',
                name='mobile_base',
                executable='mobile_base',
                parameters=[config]
         )

    find_markers_server = Node(
                package='mobile_base_unit',
                name='find_markers_server',
                executable='find_markers_server',
                output='screen'
         )

    find_markers_client = Node(
                package='mobile_base_unit',
                name='find_markers_client',
                executable='find_markers_client',
                output='screen'
         )
    

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'),

        #realsense_camera_node,
        
        #rplidar_node,    
        mobile_base_node,
        #aruco_node,
        TimerAction(
            period=2.0,
            actions=[find_markers_server]
        ),
        TimerAction(
            period=4.0,
            actions=[find_markers_client]
        ),
 
    ])
