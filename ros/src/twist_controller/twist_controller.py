import rospy
from pid import PID
from lowpass import LowPassFilter

from yaw_controller import YawController

MAX_TORQUE = 699

class Controller(object):

    # Define a class for the calculation of throttle, brake, steering.

    def __init__(self, vehicle_params):
        '''
        Claimed yaw controller for future use.
        Input for yaw controller:
            Vehicle's information of wheel base, steer ratio, maximum lateral acceleration, and maximum steer angle.
            Manually given minimum speed for angle calculation
        '''
        self.yaw_controller = YawController(
            wheel_base=vehicle_params.wheel_base,
            steer_ratio=vehicle_params.steer_ratio,
            min_speed=0.1,
            max_lat_accel=vehicle_params.max_lat_accel,
            max_steer_angle=vehicle_params.max_steer_angle)

        self.vehicle_params = vehicle_params

        '''
        Throttle controller call for function in pid.py
        return:
           throttle calculation functions
        '''
        self.throttle_controller = PID(kp=0.4, ki=0.05, kd=0.0, mn=vehicle_params.decel_limit, mx=0.50*(vehicle_params.accel_limit))
        self.brake_controller = PID(kp=100, ki=0.0, kd=1.0, mn=0.0, mx=5000)

        tau = 0.5
        ts = 0.02

        self.vel_lpf = LowPassFilter(tau, ts)
        self.last_time = rospy.get_time()
        pass

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        '''
        Args:
            current_vel (Current Velocity): data subscribed
            dbw_enabled (Whether enable dbw node): only matters when real test
            linear_vel (Linear velocity): velocity move ahead
            angular_vel (Angular velocity): yaw velocity
        Returns:
            throttle, brake, and steering values.
        '''
        brake = 0
        vel_error = 0  # needed for init

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel-current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        if vel_error > 0:
            throttle = self.throttle_controller.step(vel_error, sample_time)
            brake = 0
            self.brake_controller.reset()
        else:
            brake = self.brake_controller.step(-1.0*vel_error, sample_time)
            throttle = 0
            self.throttle_controller.reset()

        if linear_vel == 0.0 and current_vel < 0.2:   # low-speed exception to hold vehicle at rest
            throttle = 0
            brake = MAX_TORQUE

        return throttle, brake, steering
