
import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

from math import cos, sin, tan
import numpy as np
import tf

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.steer_pid = PID(kp=0.15, ki=0.05, kd=0.5, mn=-max_steer_angle, mx=max_steer_angle)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()



    def calculate_cte(self, current_pose, waypoints):
        x_vals = []
        y_vals = []

        quaternion = (current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w)
        _ig1, _ig2, yaw = tf.transformations.euler_from_quaternion(quaternion)
        originX = current_pose.pose.position.x
        originY = current_pose.pose.position.y

        for waypoint in waypoints.waypoints:

            shift_x = waypoint.pose.pose.position.x - originX
            shift_y = waypoint.pose.pose.position.y - originY

            x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
            y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

            x_vals.append(x)
            y_vals.append(y)

        coefficients = np.polyfit(x_vals, y_vals, 5)
        cte = np.polyval(coefficients, 5.0)
        return cte


    def control(self, current_pose,current_vel,target_vel, dbw_enabled, linear_vel, angular_vel, waypoints):
        # short circuit-exit if controller is disabled:
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.steer_pid.reset()
            return 0., 0., 0.

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        cte = self.calculate_cte(current_pose, waypoints)
        steering = self.steer_pid.step(cte, sample_time)

        acceleration = target_vel - current_vel/ONE_MPH
        if acceleration > 0:
            acceleration = min(self.accel_limit, acceleration)
        else:
            acceleration = max(self.decel_limit, acceleration)
        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = None, None
        # print('Target_vel = {:.2f}\nCurrent_vel = {:.2f}\naccel = {:.2f}\ntorque = {:.2f}\n'.format(target_vel,current_vel/ONE_MPH,acceleration, torque))
        if torque > 0:
            throttle, brake = min(1.0, torque / 200.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(torque), 20000.0)
        return throttle, brake, steering
