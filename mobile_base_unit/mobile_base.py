"""
    Universal Mobile Base Unit.
    This Unit has the two wheel controllers, the battery and the LIDAR.
    The responsability of the node is to read the sensors, handle common calculations 
    (filtering, odometry, inverse kinematics) and exposing control interfaces.

    The following ROS services are exposed to:
    - Get or reset the odometry (GetOdometry, ResetOdometry)
    - Get the battery voltage (GetBatteryVoltage)
    - Set an (x_vel, y_vel, theta_vel) speed command for a fixed period of time (SetSpeed)
    - Set a (x, y, theta) goal position in the odometric frame (GoToXYTheta, IsGoToFinished, DistanceToGoal)
    - Manage drive modes (GetUnitMode, SetUnitMode)
    - Enable/disable the LIDAR safety (SetUnitSafety)

    The following topics are published for:
    - Filtering the LIDAR (scan_filterd)
    - Reading the wheels rotational speed (left_wheel_rpm, right_wheel_rpm)
    - Reading the odometry (odom)

    See params.yaml for the list of ROS parameters.
"""

import os
import time
import serial
import math
import numpy as np
import traceback
import copy
import sys
from enum import Enum
from collections import deque
from csv import writer
from typing import List
from subprocess import check_output


import rclpy
import rclpy.logging
from rclpy.node import Node
import tf_transformations
from pyvesc.VESC import MultiVESC
from example_interfaces.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.constants import S_TO_NS
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from units_interfaces.srv import SetUnitMode, GetUnitMode, GetOdometry, ResetOdometry
from units_interfaces.srv import GoToXYTheta, IsGoToFinished, DistanceToGoal
from units_interfaces.srv import SetSpeed, GetBatteryVoltage, SetUnitSafety

from mobile_base_unit.utils import PID, angle_diff, sign
from mobile_base_unit.lidar_safety import LidarSafety





class UnitModes(Enum):
    """
    Unit drive modes
    CMD_VEL = The commands read on the topic /cmd_vel are applied after smoothing
    BRAKE =  Sets the PWMs to 0 effectively braking the base
    FREE_WHEEL =  Sets the current control to 0, coast mode
    SPEED =  Mode used by the set_speed service to do speed control over arbitrary duration
    GOTO =  Mode used by the go_to_xytheta service to do position control in odom frame
    EMERGENCY_STOP =  Calls the emergency_shutdown method
    """
    CMD_VEL = 1
    BRAKE = 2
    FREE_WHEEL = 3
    SPEED = 4
    GOTO = 5
    EMERGENCY_STOP = 6


class UnitControlModes(Enum):
    """
    Unit control modes
    OPEN_LOOP = The HAL will send PWM commands to the controllers.
    PID = The HAL will send speed commands to the controllers, the control is made by the low level PIDs
    """
    OPEN_LOOP = 1
    PID = 2


class MobileBase:
    """Mobile base representation and the interface with low level controllers
    """

    def __init__(
        self,
        serial_port: str = '/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00',
        left_wheel_id: int = None,
        right_wheel_id: int = 1,
    ) -> None:

        params = [
            {'can_id': left_wheel_id, 'has_sensor': True, 'start_heartbeat': True},
            {'can_id': right_wheel_id, 'has_sensor': True, 'start_heartbeat': True},
        ]
        self._multi_vesc = MultiVESC(
            serial_port=serial_port, vescs_params=params)

        self.left_wheel, self.right_wheel = self._multi_vesc.controllers
        self.left_wheel_measurements, self.right_wheel_measurements = None, None
        self.left_wheel_nones, self.right_wheel_nones = 0, 0
        self.wheel_radius = 0.20/2.0
        self.y_distance_wheels = 0.352
        self.wheel_to_center = self.y_distance_wheels/2.0
        self.half_poles = 3.5
        self.left_wheel_rpm, self.right_wheel_rpm = 0, 0
        self.left_wheel_avg_rpm, self.right_wheel_avg_rpm = 0, 0
        self.left_wheel_rpm_deque, self.right_wheel_rpm_deque  = deque([], 10), deque([], 10)
        self.battery_cell_warn_voltage = 3.7
        self.battery_cell_min_voltage = 3.5
        self.battery_nb_cells = 13
        self.battery_check_period = 60

    def read_all_measurements(self) -> None:
        """Reads all the measurements for the left, right 
        """
        self.left_wheel_measurements = self.left_wheel.get_measurements()
        self.right_wheel_measurements = self.right_wheel.get_measurements()

    def deque_to_avg(self, deque: deque) -> float:
        """Returns the average of the values contained in deque
        """
        sum = 0
        len = deque.maxlen
        for i in deque:
            sum += i
        return sum/float(len)


class MobileBaseUnit(Node):

    

    def __init__(self) -> None:
        """Node initialisation.
        ROS side: setup of the timers, callbacks, topics, services and parameters.
        Low level side: connecting with the hardware and making a first read of the sensors
        """
        super().__init__('mobile_base_unit')


        # Параметры адаптивного торможения
        self.BRAKE_RAMP_DOWN_STEPS = 5          # Шагов для плавного торможения
        self.MIN_PWM_THRESHOLD = 0.01           # Порог отключения PWM
        self.POSITION_TOLERANCE = 0.005          # 5 мм
        self.ANGLE_TOLERANCE = math.radians(0.5) # 0.5°
        self.SLOWDOWN_DISTANCE = 0.1             # 10 см - начало замедления
        self.SLOWDOWN_ANGLE = math.radians(5)    # 5° - начало замедления


        self.get_logger().info("Starting Mobile base unit!")
        self.unit_model = 0.9
        try:
            float_model = float(self.unit_model)
            if float_model < 1.0:
                self.rover_base = MobileBase(left_wheel_id=None, right_wheel_id=1)
            else:
                self.rover_base = MobileBase(left_wheel_id=None, right_wheel_id=9)
        except Exception:
            msg = "Unit version can't be processed, check that the 'unit_model'"
            self.get_logger().error(msg)
            self.get_logger().error(traceback.format_exc())
            raise RuntimeError(msg)

        self.get_logger().info(
            "Reading  sensors once...")
        self.read_measurements()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('laser_upper_angle', 2.85),
                ('laser_lower_angle', -2.85),
                ('max_duty_cycle', 0.03),
                ('cmd_vel_timeout', 0.2),
                ('max_full_com_fails', 100),
                ('main_tick_period', 0.012),
                ('control_mode', 'OPEN_LOOP'),
                ('max_accel_xy', 1.0),
                ('max_accel_theta', 1.0),
                ('xy_tol', 0.0),
                ('theta_tol', 0.0),
                ('smoothing_factor', 5.0),
                ('safety_distance', 0.85),
                ('critical_distance', 0.55),
            ])
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Making sure we don't run the node if the config file is not shared
        if self.get_parameter('max_duty_cycle').type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                "Parameter 'max_duty_cycle' was not initialized. Check that the param file is given to the node"
                "(using the launch file is the way to go). Shutting down")
            self.destroy_node()
        if self.get_parameter('smoothing_factor').type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                "Parameter 'smoothing_factor' was not initialized. Check that the param file is given to the node"
                "(using the launch file is the way to go). Shutting down")
            self.destroy_node()

        # Parameters initialisation
        self.laser_upper_angle = self.get_parameter(
            'laser_upper_angle').get_parameter_value().double_value  # math.pi
        self.laser_lower_angle = self.get_parameter(
            'laser_lower_angle').get_parameter_value().double_value  # -math.pi
        self.max_duty_cycle = self.get_parameter(
            'max_duty_cycle').get_parameter_value().double_value  # 0.03  # max is 1
        self.cmd_vel_timeout = self.get_parameter(
            'cmd_vel_timeout').get_parameter_value().double_value  # 0.2
        self.max_full_com_fails = self.get_parameter(
            'max_full_com_fails').get_parameter_value().integer_value  # 100
        self.main_tick_period = self.get_parameter(
            'main_tick_period').get_parameter_value().double_value  # 0.012


        control_mode_param = self.get_parameter('control_mode')
        if control_mode_param.value in [m.name for m in UnitControlModes]:
            # "OPEN_LOOP"
            self.control_mode = UnitControlModes[control_mode_param.value]
        else:
            self.get_logger().error(
                f"Parameter 'control_mode' has an unknown value: '{control_mode_param.value}'. Shutting down")
            self.destroy_node()

        self.max_accel_xy = self.get_parameter(
            'max_accel_xy').get_parameter_value().double_value  # 1.0
        self.max_accel_theta = self.get_parameter(
            'max_accel_theta').get_parameter_value().double_value  # 1.0
        self.xy_tol = self.get_parameter(
            'xy_tol').get_parameter_value().double_value  # 0.2
        self.theta_tol = self.get_parameter(
            'theta_tol').get_parameter_value().double_value  # 0.17
        self.smoothing_factor = self.get_parameter(
            'smoothing_factor').get_parameter_value().double_value  # 100.0
        self.safety_distance = self.get_parameter(
            'safety_distance').get_parameter_value().double_value

        self.critical_distance = self.get_parameter(
            'critical_distance').get_parameter_value().double_value
        
        self.nb_full_com_fails = 0
        self.cmd_vel = None
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0
        self.x_vel_goal = 0.0
        self.y_vel_goal = 0.0
        self.theta_vel_goal = 0.0
        self.x_vel_goal_filtered = 0.0
        self.y_vel_goal_filtered = 0.0
        self.theta_vel_goal_filtered = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.reset_odom = False
        self.battery_voltage = 48.0
        self.mode = UnitModes.CMD_VEL
        self.speed_service_deadline = 0
        self.speed_service_on = False
        self.goto_service_on = False
        self.safety_on = True

       

        self.scan_is_read = False
        self.scan_timeout = 0.5
        self.lidar_safety = LidarSafety(
            self.safety_distance, self.critical_distance, robot_collision_radius=0.6,
            speed_reduction_factor=0.88, logger=self.get_logger())

        self.x_pid = PID(p=0.5, i=0.00, d=0.0, max_command=0.5,
                         max_i_contribution=0.0)
        self.y_pid = PID(p=2.0, i=0.00, d=0.0, max_command=0.5,
                         max_i_contribution=0.0)
        self.theta_pid = PID(p=0.28, i=0.0015, d=0.20,
                             max_command=0.20, max_i_contribution=0.03)

        self.max_wheel_speed = self.pwm_to_wheel_rot_speed(self.max_duty_cycle)
        self.get_logger().info(
            f"The maximum PWM value is {self.max_duty_cycle*100}% => maximum wheel speed is set to "
            f"{self.max_wheel_speed:.2f}rad/s")

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd_vel_sub  # prevent unused variable warning... 

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_filter_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scan_sub  # prevent unused variable warning... 
        self.scan_pub = self.create_publisher(
            LaserScan, 'scan_filterd', 10)

        self.pub_left_wheel_rpm = self.create_publisher(
            Float32, 'left_wheel_rpm', 2)
        self.pub_right_wheel_rpm = self.create_publisher(
            Float32, 'right_wheel_rpm', 2)

        # 18.08.24
        self.pub_battery_voltage = self.create_publisher(
            Float32, 'battery_voltage', 10)

        self.pub_odom = self.create_publisher(
            Odometry, 'odom', 2)

        self.mode_service = self.create_service(
            SetUnitMode, 'SetUnitMode', self.handle_unit_mode)

        self.get_mode_service = self.create_service(
            GetUnitMode, 'GetUnitMode', self.handle_get_unit_mode)

        self.reset_odometry_service = self.create_service(
            ResetOdometry, 'ResetOdometry', self.handle_reset_odometry)

        self.get_odometry_service = self.create_service(
            GetOdometry, 'GetOdometry', self.handle_get_odometry)

        self.set_speed_service = self.create_service(
            SetSpeed, 'SetSpeed', self.handle_set_speed)

        
        self.go_to_service = self.create_service(
            GoToXYTheta, 'GoToXYTheta', self.handle_go_to)

        self.is_go_to_finished = self.create_service(
            IsGoToFinished, 'IsGoToFinished', self.handle_is_go_to_finished)

        self.distance_to_goal = self.create_service(
            DistanceToGoal, 'DistanceToGoal', self.handle_distance_to_goal)

        self.get_battery_voltage_service = self.create_service(
            GetBatteryVoltage, 'GetBatteryVoltage', self.handle_get_battery_voltage)

        self.set_safety_service = self.create_service(
            SetUnitSafety, 'SetUnitSafety', self.handle_unit_set_safety)

        
        # self.set_front_lights_service = self.create_service(
        #     SetFrontLights, 'SetFrontLights', self.handle_set_front_lights)

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.old_measure_timestamp = self.get_clock().now()
        self.measure_timestamp = self.get_clock().now()
        # *sigh* if needed use: https://github.com/ros2/rclpy/blob/master/rclpy/test/test_time.py
        self.cmd_vel_t0 = time.time()
        self.scan_t0 = time.time()
        self.t0 = time.time()
        self.read_measurements()
        self.first_tick = True

        self.create_timer(self.main_tick_period, self.main_tick)
        # self.create_timer(0.1, self.main_tick)
        self.measurements_t = time.time()
        # Checking battery once at the start, then periodically
        self.check_battery()
        self.create_timer(self.rover_base.battery_check_period,
                          self.check_battery)
    
    def _enable_brake_mode(self):
        """Плавное торможение с использованием констант"""
        current_pwm = max(
            abs(self.rover_base.left_wheel_measurements.duty_cycle_now),
            abs(self.rover_base.right_wheel_measurements.duty_cycle_now)
        )
        
        # Плавное снижение PWM
        for step in range(self.BRAKE_RAMP_DOWN_STEPS):
            ratio = (self.BRAKE_RAMP_DOWN_STEPS - step) / self.BRAKE_RAMP_DOWN_STEPS
            pwm = current_pwm * ratio
            pwm = max(pwm, self.MIN_PWM_THRESHOLD)  # Не опускаться ниже порога
            
            self.rover_base.left_wheel.set_duty_cycle(pwm)
            self.rover_base.right_wheel.set_duty_cycle(pwm)
            time.sleep(0.01)
        
        # Финишное торможение
        self.rover_base.left_wheel.set_duty_cycle(0)
        self.rover_base.right_wheel.set_duty_cycle(0)
        self.mode = UnitModes.BRAKE

    def _check_position_error(self) -> bool:
        """Проверка достижения цели с использованием констант"""
        pos_error = math.sqrt((self.x_goal - self.x_odom)**2 + 
                            (self.y_goal - self.y_odom)**2)
        angle_error = abs(angle_diff(self.theta_goal, self.theta_odom))
        
        return (pos_error < self.POSITION_TOLERANCE and 
                angle_error < self.ANGLE_TOLERANCE)


    def parameters_callback(self, params) -> None:
        """When a ROS parameter is changed, this method will be called to verify the change and accept/deny it.

        """
        success = False
        for param in params:
            if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                if param.name == "laser_upper_angle":
                    self.laser_upper_angle = param.value
                    success = True
                elif param.name == "laser_lower_angle":
                    self.laser_lower_angle = param.value
                    success = True
                elif param.name == "max_duty_cycle":
                    if param.value >= 0.0 and param.value <= 1.0:
                        self.max_duty_cycle = param.value
                        success = True
                elif param.name == "cmd_vel_timeout":
                    if param.value >= 0.0:
                        self.cmd_vel_timeout = param.value
                        success = True
                elif param.name == "max_full_com_fails":
                    if param.value >= 0.0:
                        self.max_full_com_fails = param.value
                        success = True
                elif param.name == "main_tick_period":
                    if param.value >= 0.0:
                        self.main_tick_period = param.value
                        success = True
                elif param.name == "max_accel_xy":
                    if param.value >= 0.0:
                        self.max_accel_xy = param.value
                        success = True
                elif param.name == "max_accel_theta":
                    if param.value >= 0.0:
                        self.max_accel_theta = param.value
                        success = True
                elif param.name == "xy_tol":
                    if param.value >= 0.0:
                        self.xy_tol = param.value
                        success = True
                elif param.name == "theta_tol":
                    if param.value >= 0.0:
                        self.theta_tol = param.value
                        success = True
                elif param.name == "smoothing_factor":
                    if param.value >= 0.0:
                        self.smoothing_factor = param.value
                        success = True
                elif param.name == "safety_distance":
                    if param.value >= 0.0:
                        self.safety_distance = param.value
                        success = True
                elif param.name == "critical_distance":
                    if param.value >= 0.0:
                        self.critical_distance = param.value
                        success = True

            elif param.type_ is Parameter.Type.STRING:
                if param.name == "control_mode":
                    if param.value in [m.name for m in UnitControlModes]:
                        self.control_mode = UnitControlModes[param.value]
                        success = True

        return SetParametersResult(successful=success)

    def handle_unit_mode(self, request: SetUnitMode.Request, response: SetUnitMode.Response
                         ) -> SetUnitMode.Response:
        """Handle SetUnitMode service request"""
        self.get_logger().info("Requested mode change to :'{}'".format(request.mode))
        response.success = False

        if request.mode in [m.name for m in UnitModes]:
            if request.mode == UnitModes.SPEED.name:
                self.get_logger().info("'{}' should not be changed by hand, use the SetSpeed service instead"
                                       .format(request.mode))
            elif request.mode == UnitModes.GOTO.name:
                self.get_logger().info("'{}' should not be changed by hand, use the GoToXYTheta service instead"
                                       .format(request.mode))
            else:
                # Changing the mode is a way to prematurely end an on going task requested through a service
                self.stop_ongoing_services()
                self.mode = UnitModes[request.mode]
                response.success = True
                self.get_logger().info("OK")

        return response

    def handle_get_unit_mode(self, request: GetUnitMode.Request, response: GetUnitMode.Response
                             ) -> GetUnitMode.Response:
        """Handle GetUnitMode service request"""
        response.mode = self.mode.name
        return response

    def handle_reset_odometry(self, request: ResetOdometry.Request, response: ResetOdometry.Response
                              ) -> ResetOdometry.Response:
        """Handle ResetOdometry service request"""
        # Resetting asynchronously to prevent race conditions.
        self.get_logger().info("Requested to reset the odometry frame")

        self.read_measurements()  # Принудительно читаем новые данные
        self.update_wheel_speeds()  # Обновляем rpm
        time.sleep(0.05)  # Небольшая задержка для стабилизации
        self.reset_odom = True
        response.success = True
        return response

    def handle_get_odometry(self, request: GetOdometry.Request, response: GetOdometry.Response
                            ) -> GetOdometry.Response:
        response.x = self.x_odom
        response.y = self.y_odom
        response.theta = self.theta_odom
        return response

    def handle_set_speed(self, request: SetSpeed.Request, response: SetSpeed.Response
                         ) -> SetSpeed.Response:
        """Handle SetSpeed service request"""
        # This service automatically changes the Unit mode
        self.mode = UnitModes.SPEED
        self.get_logger().info(
            f"Requested set_speed: duration={request.duration} x_vel='{request.x_vel}'m/s, y_vel='{request.y_vel}'m/s,"
            f"rot_vel='{request.rot_vel}'rad/s")
        self.x_vel_goal = request.x_vel
        self.y_vel_goal = request.y_vel
        self.theta_vel_goal = request.rot_vel
        self.speed_service_deadline = time.time() + request.duration
        self.speed_service_on = True
        response.success = True
        return response

    def handle_go_to(self, request: GoToXYTheta.Request, response: GoToXYTheta.Response
                     ) -> GoToXYTheta.Response:
        """Handle GoToXYTheta service request"""
        # This service automatically changes the Unit mode
        self.mode = UnitModes.GOTO
        self.get_logger().info(
            f"Requested go_to: x={request.x_goal}m, y={request.y_goal}m, theta={request.theta_goal}rad")
        self.x_goal = request.x_goal
        self.y_goal = request.y_goal
        self.theta_goal = request.theta_goal
        self.goto_service_on = True
        self.x_pid.set_goal(self.x_goal)
        self.y_pid.set_goal(self.y_goal)
        self.theta_pid.set_goal(self.theta_goal)
        response.success = True
        return response

    def handle_is_go_to_finished(self, request: IsGoToFinished.Request, response: IsGoToFinished.Response
                                 ) -> IsGoToFinished.Response:
        """Handle IsGoToFinished service request"""
        # Returns True if the goal position is reached
        response.success = self.goto_service_on
        return response

    def handle_distance_to_goal(self, request: DistanceToGoal.Request, response: DistanceToGoal.Response
                                ) -> DistanceToGoal.Response:
        """Handle DistanceToGoal service resquest"""
        response.delta_x = self.x_goal - self.x_odom
        response.delta_y = self.y_goal - self.y_odom
        response.delta_theta = angle_diff(self.theta_goal, self.theta_odom)
        response.distance = math.sqrt(
            (self.x_goal - self.x_odom)**2 +
            (self.y_goal - self.y_odom)**2)
        return response

    def handle_get_battery_voltage(self, request: GetBatteryVoltage.Request, response: GetBatteryVoltage.Response
                                   ) -> GetBatteryVoltage.Response:
        """Handle GetBatteryVoltage service request"""
        response.voltage = self.battery_voltage
        return response

    def handle_unit_set_safety(self, request: SetUnitSafety.Request, response: SetUnitSafety.Response
                               ) -> SetUnitSafety.Response:
        """Handle SetUnitSafety service request"""
        safety_on = request.safety_on
        state = 'ON' if safety_on else 'OFF'
        self.get_logger().info(f"Lidar safety is now {state}")
        self.safety_on = safety_on
        response.success = True
        return response
    """
    def handle_set_front_lights(self, request: SetFrontLights.Request, response: SetFrontLights.Response
                               ) -> SetFrontLights.Response:
    


        front_lights_on = request.front_lights_on
        
        if front_lights_on: 
            GPIO.output(Lights.FRONT_RIGHT, GPIO.HIGH)
            GPIO.output(Lights.FRONT_LEFT, GPIO.HIGH)
            state = 'ON'
        else: 
            GPIO.output(Lights.FRONT_RIGHT, GPIO.LOW)
            GPIO.output(Lights.LEFT_LIGHT, GPIO.LOW)
            state = 'OFF'

        self.get_logger().info(f"Front lights are now {state}")
        self.front_lights_on = front_lights_on
        response.success = True
        return response
    """


    def check_battery(self, verbose: bool = False) -> None:
        """Checks that the battery readings are not too old and forces a read if need be.
        Checks that the battery voltages are safe and warns or stops the HAL accordingly.
        """
        t = time.time()
        # 18.08.24
        voltage_pub = Float32()

        if verbose:
            self.print_all_measurements()
        if (t - self.measurements_t) > (self.rover_base.battery_check_period+1):
            self.get_logger().warning("Unit's measurements are not made often enough. Reading now.")
            self.read_measurements()
        warn_voltage = self.rover_base.battery_nb_cells * \
            self.rover_base.battery_cell_warn_voltage
        min_voltage = self.rover_base.battery_nb_cells * \
            self.rover_base.battery_cell_min_voltage
        voltage = self.battery_voltage

        # 18.08.24
        voltage_pub.data = float(self.battery_voltage)
        self.pub_battery_voltage.publish(voltage_pub)

        if (min_voltage < voltage < warn_voltage):
            self.get_logger().warning("Battery voltage LOW ({}V). Consider recharging. Warning threshold: {:.1f}V, "
                                      "stop threshold: {:.1f}V".format(voltage, warn_voltage, min_voltage))
        elif (voltage < min_voltage):
            msg = "Battery voltage critically LOW ({}V). Emergency shutdown! Warning threshold: {:.1f}V, "
            "stop threshold: {:.1f}V".format(voltage, warn_voltage, min_voltage)
            self.get_logger().error(msg)
            self.emergency_shutdown()
            raise RuntimeError(msg)
        else:
            self.get_logger().warning("Battery voltage OK ({}V)".format(voltage))

    def emergency_shutdown(self) -> None:
        """Sets a PWM of 0V to the three wheel motors, equivalent to a brake.
        """
        self.rover_base.left_wheel.set_duty_cycle(0)
        self.rover_base.right_wheel.set_duty_cycle(0)
        self.get_logger().warn("Emergency shutdown!")
        time.sleep(0.1)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Handles the callback on the /cmd_vel topic
        """
        self.cmd_vel = msg
        self.cmd_vel_t0 = time.time()

    def scan_filter_callback(self, msg: LaserScan) -> None:
        """Callback method on the /scan topic. Handles the LIDAR filtering and safety calculations.
        """
        self.scan_is_read = True
        self.scan_t0 = time.time()
        # LIDAR angle filter managemnt
        # Temporary fix since https://github.com/ros-perception/laser_filters doesn't work on Foxy at the moment
        filtered_scan = LaserScan()
        filtered_scan.header = copy.deepcopy(msg.header)
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        ranges = []
        intensities = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment
            if angle > self.laser_upper_angle or angle < self.laser_lower_angle:
                ranges.append(0.0)
                intensities.append(0.0)
            else:
                ranges.append(r)
                intensities.append(msg.intensities[i])

        filtered_scan.ranges = ranges
        filtered_scan.intensities = intensities
        self.scan_pub.publish(filtered_scan)

        # LIDAR safety management
        self.lidar_safety.clear_measures()
        if self.safety_on:
            self.lidar_safety.process_scan(filtered_scan)

    def wheel_rot_speed_to_pwm_no_friction(self, rot: float) -> float:
        """Uses a simple linear model to map the expected rotational speed of the wheel to a constant PWM
        (based on measures made on a full Reachy Mobile)
        """
        return rot/22.7

    def wheel_rot_speed_to_pwm(self, rot: float) -> float:
        """Uses a simple affine model to map the expected rotational speed of the wheel to a constant PWM
        (based on measures made on a full Reachy Mobile)
        """
        # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
        epsilon = 0.02
        if rot > epsilon:
            pwm = 0.0418 * rot + 0.0126
        elif rot < -epsilon:
            pwm = 0.0418 * rot - 0.0126
        else:
            pwm = 0.0

        return pwm

    def pwm_to_wheel_rot_speed(self, pwm: float) -> float:
        """Uses a simple affine model to map a pwm to the expected rotational speed of the wheel
        (based on measures made on a full Reachy Mobile)
        """
        # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
        if abs(pwm) < 0.0126:
            rot = 0.0
        else:
            rot = sign(pwm)*(abs(pwm)-0.0126)/0.0418

        return rot

    def ik_vel_to_pwm(self, x_vel: float, y_vel: float, rot_vel: float) -> List[float]:
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame)
        and outputs the PWM to apply to each of the 3 motors in an omni setup

        Args:
            x (float): x speed (m/s). Positive "in front" of the robot.
            y (float): y speed (m/s). Positive "to the left" of the robot.
            rot (float): rotational speed (rad/s). Positive counter-clock wise.

        """
        rot_vels = self.ik_vel(x_vel, y_vel, rot_vel)
        return [self.wheel_rot_speed_to_pwm(rot_vel) for rot_vel in rot_vels]

    def ik_vel_old(self, x: float, y: float, rot: float) -> List[float]:
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame)
        and outputs the PWM to apply to each of the 3 motors in an omni setup

        Args:
            x (float): x speed (between -1 and 1). Positive "in front" of the robot.
            y (float): y speed (between -1 and 1). Positive "to the left" of the robot.
            rot (float): rotational speed (between -1 and 1). Positive counter-clock wise.
        """
        cycle_back = -y + rot
        cycle_right = (-y*np.cos(120*np.pi/180)) + \
            (x*np.sin(120*np.pi/180)) + rot
        cycle_left = (-y*np.cos(240*np.pi/180)) + \
            (x*np.sin(240*np.pi/180)) + rot

        return [cycle_back, cycle_right, cycle_left]

    # Interesting stuff can be found in Modern Robotics' chapters:
    # 13.2 Omnidirectional Wheeled Mobile Robots
    # 13.4 Odometry
    # /!\ Our robot frame is different. Matching between their names (left) and ours (right):
    # xb=y, yb=-x, theta=-theta, u1=uB, u2=uL, u3=uR


    def ik_vel(self, x_vel: float, y_vel: float, rot_vel: float) -> List[float]:
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame) and outputs the rotational speed (rad/s)
        of each of the 3 motors in an omni setup

        Args:
            x (float): x speed (m/s). Positive "in front" of the robot.
            y (float): y speed (m/s). Positive "to the left" of the robot.
            rot (float): rotational speed (rad/s). Positive counter-clock wise.
        """
        wheel_rot_speed_back = 0
        wheel_rot_speed_right = (x_vel + rot_vel * self.rover_base.y_distance_wheels/2) / self.rover_base.wheel_radius
        wheel_rot_speed_left = (x_vel - rot_vel * self.rover_base.y_distance_wheels/2) / self.rover_base.wheel_radius

        return [wheel_rot_speed_back, wheel_rot_speed_right, wheel_rot_speed_left]




    def dk_vel(self, rot_l: float, rot_r: float) -> float:
        """Takes the 2 rotational speeds (in rpm) of the 2 wheels and outputs the x linear speed (m/s),
        y linear speed (m/s) and rotational speed (rad/s) in the robot egocentric frame

        Args:
            rot_l (float): rpm speed of the left wheel
            rot_r (float): rpm speed of the right wheel

        """
        # rpm to rad/s then m/s
    
        speed_l = (2*math.pi*rot_l/60)*self.rover_base.wheel_radius
        speed_r = (2*math.pi*rot_r/60)*self.rover_base.wheel_radius
        
        """   
        speed_l = math.radians(rot_l/60)
        speed_r = math.radians(rot_r/60)
        """
        x_vel = (self.rover_base.wheel_radius * (speed_r + speed_l)) / 2
        y_vel = 0.0
        theta_vel = (self.rover_base.wheel_radius * (speed_r - speed_l)) / self.rover_base.y_distance_wheels

        return [x_vel, y_vel, theta_vel]

    def filter_speed_goals(self) -> List[float]:
        """Applies a smoothing filter on x_vel_goal, y_vel_goal and theta_vel_goal
        """
        self.x_vel_goal_filtered = (self.x_vel_goal +
                                    self.smoothing_factor*self.x_vel_goal_filtered)/(1+self.smoothing_factor)
        self.y_vel_goal_filtered = (self.y_vel_goal +
                                    self.smoothing_factor*self.y_vel_goal_filtered)/(1+self.smoothing_factor)
        self.theta_vel_goal_filtered = (self.theta_vel_goal +
                                        self.smoothing_factor*self.theta_vel_goal_filtered)/(1+self.smoothing_factor)

        self.x_vel_goal_filtered, \
            self.y_vel_goal_filtered, \
            self.theta_vel_goal_filtered = self.lidar_safety.safety_check_speed_command(
                self.x_vel_goal_filtered, self.y_vel_goal_filtered, self.theta_vel_goal_filtered)

    def format_measurements(self, measurements) -> str:
        """Text formatting for the low level controller measurements
        """
        if measurements is None:
            return "None"
        to_print = ""
        to_print += "temp_fet:{}\n".format(measurements.temp_fet)
        to_print += "temp_motor:{}\n".format(measurements.temp_motor)
        to_print += "avg_motor_current:{}\n".format(
            measurements.avg_motor_current)
        to_print += "avg_input_current:{}\n".format(
            measurements.avg_input_current)
        to_print += "avg_id:{}\n".format(measurements.avg_id)
        to_print += "avg_iq:{}\n".format(measurements.avg_iq)
        to_print += "duty_cycle_now:{}\n".format(measurements.duty_cycle_now)
        to_print += "rpm:{}\n".format(measurements.rpm)
        to_print += "v_in:{}\n".format(measurements.v_in)
        to_print += "amp_hours:{}\n".format(measurements.amp_hours)
        to_print += "amp_hours_charged:{}\n".format(
            measurements.amp_hours_charged)
        to_print += "watt_hours:{}\n".format(measurements.watt_hours)
        to_print += "watt_hours_charged:{}\n".format(
            measurements.watt_hours_charged)
        to_print += "tachometer:{}\n".format(measurements.tachometer)
        to_print += "tachometer_abs:{}\n".format(measurements.tachometer_abs)
        to_print += "mc_fault_code:{}\n".format(measurements.mc_fault_code)
        to_print += "pid_pos_now:{}\n".format(measurements.pid_pos_now)
        to_print += "app_controller_id:{}\n".format(
            measurements.app_controller_id)
        to_print += "time_ms:{}\n".format(measurements.time_ms)
        return to_print

    def print_all_measurements(self) -> None:
        """Prints the low level measurements from the 3 wheel controllers
        """
        to_print = "\n*** back_wheel measurements:\n"
        to_print += self.format_measurements(
            self.rover_base.back_wheel_measurements)
        to_print += "\n\n*** left_wheel:\n"
        to_print += self.format_measurements(
            self.rover_base.left_wheel_measurements)
        to_print += "\n\n*** right_wheel:\n"
        to_print += self.format_measurements(
            self.rover_base.right_wheel_measurements)
        to_print += "\n\n Fails ('Nones') left:{}, right:{}, back:{}".format(
            self.rover_base.left_wheel_nones, self.rover_base.right_wheel_nones, self.rover_base.back_wheel_nones)
        to_print += "\n\n AVG RPM left:{:.2f}, right:{:.2f}, back:{:.2f}".format(
            self.rover_base.left_wheel_avg_rpm/self.rover_base.half_poles,
            self.rover_base.right_wheel_avg_rpm/self.rover_base.half_poles,
            self.rover_base.back_wheel_avg_rpm/self.rover_base.half_poles)

        self.get_logger().info("{}".format(to_print))
        # 20 tours en 35s, avg_rpm ~=34

    def publish_wheel_speeds(self) -> None:
        """Publishes the most recent measure of rotational speed for each of the 3 wheels on 2 separate topics.
        """
        # If the measurements are None, not publishing
        
        if self.rover_base.left_wheel_measurements is not None:
            rpm_left = Float32()
            rpm_left.data = float(self.rover_base.left_wheel_measurements.rpm)
            self.pub_left_wheel_rpm.publish(rpm_left)

        if self.rover_base.right_wheel_measurements is not None:
            rpm_right = Float32()
            rpm_right.data = float(self.rover_base.right_wheel_measurements.rpm)
            self.pub_right_wheel_rpm.publish(rpm_right)

    def update_wheel_speeds(self) -> None:
        """Uses the latest mesure of wheel rotational speed to update the smoothed internal estimation of the wheel
        rotational speed
        """
        # Keeping a local value of the wheel speeds to handle None measurements (we'll use the last valid measure)

        self.rover_base.right_wheel_nones = 0
        self.rover_base.left_wheel_nones = 0
        

        #if self.rover_base.back_wheel_measurements is not None:
        #    value = float(
        #       self.rover_base.back_wheel_measurements.rpm)
        #  self.rover_base.back_wheel_rpm = value
        #  self.rover_base.back_wheel_rpm_deque.appendleft(value)
        #   self.rover_base.back_wheel_avg_rpm = self.rover_base.deque_to_avg(
        #       self.rover_base.back_wheel_rpm_deque)
        # else:
        #    self.rover_base.back_wheel_nones += 1

        if self.rover_base.left_wheel_measurements is not None:
            value = float(
                self.rover_base.left_wheel_measurements.rpm)
            self.rover_base.left_wheel_rpm = value
            self.rover_base.left_wheel_rpm_deque.appendleft(value)
            self.rover_base.left_wheel_avg_rpm = self.rover_base.deque_to_avg(
                self.rover_base.left_wheel_rpm_deque)
        else:
            self.rover_base.left_wheel_nones += 1

        if self.rover_base.right_wheel_measurements is not None:
            value = float(
                self.rover_base.right_wheel_measurements.rpm)
            self.rover_base.right_wheel_rpm = value
            self.rover_base.right_wheel_rpm_deque.appendleft(value)
            self.rover_base.right_wheel_avg_rpm = self.rover_base.deque_to_avg(
                self.rover_base.right_wheel_rpm_deque)
        else:
            self.rover_base.right_wheel_nones += 1

    def publish_odometry_and_tf(self) -> None:
        """Publishes the current odometry position (Odometry type published on the /odom topic) and also
        published the TransformStamped between the frame base_footprint and odom
        """
        # Odom
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.measure_timestamp.to_msg()
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x_odom
        odom.pose.pose.position.y = self.y_odom
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_odom)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # TODO tune these numbers if needed
        odom.pose.covariance = np.diag(
            [1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        odom.twist.twist.linear.x = self.x_vel
        odom.twist.twist.linear.y = self.y_vel
        odom.twist.twist.angular.z = self.theta_vel

        odom.twist.covariance = np.diag(
            [1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
        self.pub_odom.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = self.measure_timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x_odom
        t.transform.translation.y = self.y_odom
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def tick_odom(self) -> None:
        """Updates the odometry values based on the small displacement measured since the last tick, 
        then published the results with publish_odometry_and_tf()
        """
        # TODO VESC speed values are, as is normal, very noisy at low speeds.
        # We have no control on how the speeds are calculated as is.
        # -> By reading the encoder ticks directly and making the calculations here we could make this a tad better.

        # Local speeds in egocentric frame.
        # "rpm" are actually erpm and need to be divided by half the amount of magnetic poles to get the actual rpm.
        
        if self.reset_odom:
            self.reset_odom = False
            self.x_odom = 0.0
            self.y_odom = 0.0
            self.theta_odom = 0.0
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.theta_vel = 0.0
            self.rover_base.left_wheel_rpm_deque.clear()
            self.rover_base.right_wheel_rpm_deque.clear()
            if self.goto_service_on:
                self.goto_service_on = False
            return  # Пропустить этот тик
        self.x_vel, self.y_vel, self.theta_vel = self.dk_vel(self.rover_base.left_wheel_rpm/self.rover_base.half_poles,
                                                            self.rover_base.right_wheel_rpm/self.rover_base.half_poles)
        # self.get_logger().info(
        #     "IK vel : {:.2f}, {:.2f}, {:.2f}".format(self.x_vel, self.y_vel, self.theta_vel))
        # Applying the small displacement in the world-fixed odom frame (simple 2D rotation)
        dt_duration = (self.measure_timestamp - self.old_measure_timestamp)
        dt_seconds = dt_duration.nanoseconds/S_TO_NS
        dx = (self.x_vel * math.cos(self.theta_odom) - self.y_vel *
              math.sin(self.theta_odom)) * dt_seconds
        dy = (self.x_vel * math.sin(self.theta_odom) + self.y_vel *
              math.cos(self.theta_odom)) * dt_seconds
        dtheta = self.theta_vel*dt_seconds
        self.x_odom += dx
        self.y_odom += dy
        self.theta_odom += dtheta

        self.vx = dx / dt_seconds
        self.vy = dy / dt_seconds
        self.vtheta = dtheta / dt_seconds

        self.publish_odometry_and_tf()

    def limit_duty_cycles(self, duty_cycles: List[float]) -> List[float]:
        """Limits the duty cycles to stay in +-max_duty_cycle
        """
        for i in range(len(duty_cycles)):
            if duty_cycles[i] < 0:
                duty_cycles[i] = max(-self.max_duty_cycle, duty_cycles[i])
            else:
                duty_cycles[i] = min(self.max_duty_cycle, duty_cycles[i])
        return duty_cycles

    def limit_wheel_speeds(self, wheel_speeds: List[float]) -> List[float]:
        """Limits the wheel speeds to stay in +-max_wheel_speed
        """
        for i in range(len(wheel_speeds)):
            if wheel_speeds[i] < 0:
                wheel_speeds[i] = max(-self.max_wheel_speed, wheel_speeds[i])
            else:
                wheel_speeds[i] = min(
                    self.max_wheel_speed, wheel_speeds[i])
        return wheel_speeds

    def read_measurements(self) -> None:
        """Calls the low level functions to read the measurements on the 3 wheel controllers
        """
        self.nb_full_com_fails = 0
        self.max_full_com_fails = 100
        self.rover_base.read_all_measurements()
        #if self.rover_base.back_wheel_measurements is not None:
        #   self.battery_voltage = self.rover_base.back_wheel_measurements.v_in
        if self.rover_base.left_wheel_measurements is not None:
            self.battery_voltage = self.rover_base.left_wheel_measurements.v_in
        elif self.rover_base.right_wheel_measurements is not None:
            self.battery_voltage = self.rover_base.right_wheel_measurements.v_in
        else:
            # Decidemment ! Keeping last valid measure...
            self.nb_full_com_fails += 1
            # self.get_logger().warning(
            #     "Could not read any of the motor drivers. This should not happen too often.")
            if (self.nb_full_com_fails > self.max_full_com_fails): 

                msg = "Too many communication errors, emergency shutdown"
                self.get_logger().error(msg)
                self.emergency_shutdown()
                raise RuntimeError(msg)
            return
        # Read success
        self.nb_full_com_fails = 0
        self.measurements_t = time.time()

    def send_wheel_commands(self, wheel_speeds: List[float]) -> None:
        """Sends either a PWM command or a speed command to the wheel controllers, based on the current control mode"""
        if self.control_mode is UnitControlModes.OPEN_LOOP:
            duty_cycles = [self.wheel_rot_speed_to_pwm(
                wheel_speed) for wheel_speed in wheel_speeds]
            duty_cycles = self.limit_duty_cycles(duty_cycles)
            self.rover_base.left_wheel.set_duty_cycle(
                duty_cycles[2])
            self.rover_base.right_wheel.set_duty_cycle(
                duty_cycles[1])
        elif self.control_mode is UnitControlModes.PID:
            # rad/s to rpm to erpm
            wheel_speeds = self.limit_wheel_speeds(wheel_speeds)
    
            self.rover_base.left_wheel.set_rpm(
                int(self.rover_base.half_poles*wheel_speeds[2]*30/math.pi))
            self.rover_base.right_wheel.set_rpm( 
                int(self.rover_base.half_poles*wheel_speeds[1]*30/math.pi))
        else:
            self.get_logger().warning("unknown control mode '{}'".format(self.control_mode))

    def position_control(self) -> List[float]:
        """Calculates the speed targets to be sent on x, y and theta to reach their respective goals during a GoTo
        This function uses 3 separate PIDs controllers.
        """
        # x_command_odom = self.x_pid.tick(self.x_odom)
        # y_command_odom = self.y_pid.tick(self.y_odom)
        # theta_command_odom = self.theta_pid.tick(
        #     self.theta_odom, is_angle=True)

        # x_command = x_command_odom * \
        #     math.cos(-self.theta_odom) - y_command_odom * \
        #     math.sin(-self.theta_odom)
        # y_command = x_command_odom * \
        #     math.sin(-self.theta_odom) + y_command_odom * \
        #     math.cos(-self.theta_odom)

        # return x_command, y_command, theta_command_odom

        if self._check_position_error():
            self._enable_brake_mode()
            return 0.0, 0.0, 0.0
    
        # Адаптивное управление
        error = math.sqrt((self.x_goal - self.x_odom)**2 + (self.y_goal - self.y_odom)**2)
        angle_error = abs(angle_diff(self.theta_goal, self.theta_odom))
        self.get_logger().info(f"Angle error: {math.degrees(angle_error):.2f}°")  # Логируем ошибку в градусах

        # Плавное снижение мощности у цели
        if error < 0.1 or angle_error < math.radians(5):
            self.theta_pid.max_command = 0.15
            self.x_pid.max_command = 0.1
            self.y_pid.max_command = 0.1
        else:
            self.theta_pid.max_command = 0.3
            self.x_pid.max_command = 0.3    
            self.y_pid.max_command = 0.3

        return self.x_pid.tick(self.x_odom), self.y_pid.tick(self.y_odom), self.theta_pid.tick(self.theta_odom, is_angle=True)

    def stop_ongoing_services(self) -> None:
        """Stops the GoTo and the SetSpeed services, if they were running"""
        self.goto_service_on = False
        self.speed_service_on = False

    def main_tick(self, verbose: bool = False):
        """Main function of the Mobile base unit node. This function is made to be called often. Handles the main state machine"""
        t = time.time()
        """
        if (not self.scan_is_read) or ((t - self.scan_t0) > self.scan_timeout):
            # If too much time without a LIDAR scan, the speeds are set to 0 for safety.
            self.get_logger().warning(
                "waiting for a LIDAR scan to be read. Discarding all commands...")
            wheel_speeds = self.ik_vel(0.0, 0.0, 0.0)
            self.send_wheel_commands(wheel_speeds)
            time.sleep(0.5)
            return
        """
        if self.first_tick:
            self.first_tick = False
            self.get_logger().info("=> Mobile base unit up and running! **")

        if self.mode is UnitModes.CMD_VEL:
            # If too much time without an order, the speeds are smoothed back to 0 for safety.
            if (self.cmd_vel is not None) and ((t - self.cmd_vel_t0) < self.cmd_vel_timeout):
                self.x_vel_goal = self.cmd_vel.linear.x
                self.y_vel_goal = self.cmd_vel.linear.y
                self.theta_vel_goal = self.cmd_vel.angular.z
            else:
                self.x_vel_goal = 0.0
                self.y_vel_goal = 0.0
                self.theta_vel_goal = 0.0
            self.filter_speed_goals()
            # Applying the LIDAR safety
            wheel_speeds = self.ik_vel(
                self.x_vel_goal_filtered, self.y_vel_goal_filtered, self.theta_vel_goal_filtered)
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is UnitModes.BRAKE:
            self.rover_base.left_wheel.set_duty_cycle(0)
            self.rover_base.right_wheel.set_duty_cycle(0)
        elif self.mode is UnitModes.FREE_WHEEL:
            self.rover_base.left_wheel.set_current(0)
            self.rover_base.right_wheel.set_current(0)
        elif self.mode is UnitModes.SPEED:
            if self.speed_service_deadline < time.time():
                if self.speed_service_on:
                    self.get_logger().info("End of set speed duration, setting speeds to 0")
                self.speed_service_on = False
                self.x_vel_goal = 0
                self.y_vel_goal = 0
                self.theta_vel_goal = 0
            self.filter_speed_goals()
            wheel_speeds = self.ik_vel(
                self.x_vel_goal_filtered, self.y_vel_goal_filtered, self.theta_vel_goal_filtered)
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is UnitModes.GOTO:

            x_vel, y_vel, theta_vel = 0, 0, 0
            # if self.goto_service_on:
            #     distance = math.sqrt(
            #         (self.x_goal - self.x_odom)**2 +
            #         (self.y_goal - self.y_odom)**2
            #     )
            #     if distance < self.xy_tol and abs(angle_diff(self.theta_goal, self.theta_odom)) < self.theta_tol:
            #         self.goto_service_on = False
            #     else:
            #         x_vel, y_vel, theta_vel = self.position_control()

            if not self.goto_service_on:
                self._enable_brake_mode()  # Гарантированное торможение при отмене
            else:
                x_vel, y_vel, theta_vel = self.position_control()
                wheel_speeds = self.ik_vel(x_vel, y_vel, theta_vel)
                self.send_wheel_commands(wheel_speeds)

            x_vel, y_vel, theta_vel = self.lidar_safety.safety_check_speed_command(
                x_vel, y_vel, theta_vel)
            wheel_speeds = self.ik_vel(
                x_vel, y_vel, theta_vel)
            self.send_wheel_commands(wheel_speeds)

        elif self.mode is UnitModes.EMERGENCY_STOP:
            msg = "Emergency stop requested"
            self.get_logger().warning(msg)
            self.emergency_shutdown()
            raise RuntimeError(msg)

        else:
            self.get_logger().warning("unknown mode '{}', setting it to brake".format(self.mode))
            self.mode = UnitModes.BRAKE

        self.old_measure_timestamp = self.measure_timestamp
        self.measure_timestamp = self.get_clock().now()

        # Reading the measurements (this is what takes most of the time, ~9ms)
        self.read_measurements()
        self.update_wheel_speeds()

        if verbose:
            self.print_all_measurements()

        self.publish_wheel_speeds()
        self.tick_odom()

        if verbose:
            self.get_logger().info("x_odom {}, y_odom {}, theta_odom {}".format(
                self.x_odom, self.y_odom, self.theta_odom))

        # Time measurement
        dt = time.time() - t
        if dt == 0:
            f = 0
        else:
            f = 1/dt
        if verbose:
            self.get_logger().info(
                "unit tick potential freq: {:.0f}Hz (dt={:.0f}ms)".format(f, 1000*dt))


def main(args=None) -> None:
    """Run Mobile base unit main loop"""
    rclpy.init(args=args)
    try:
        node = MobileBaseUnit()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init Mobile base unit')
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
