
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
from math import sqrt


class Controller(object):
    def __init__(self, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio,
			max_lat_accel, max_steer_angle):

	self.min_speed = ONE_MPH
	self.sample_time = .02 # where looping 50 hz

        self.pid_steer = PID(kp=1.0, ki=0.00001, kd=1.0)
	self.pid_speed = PID(kp=1.0, ki=0.00001, kd=1.0)
	self.lowpass = LowPassFilter(tau=1.0, ts=1.0)
	self.yaw_controller = YawController(wheel_base, steer_ratio, self.min_speed,
						max_lat_accel, max_steer_angle)
        return

    def control(self, proposed_linear_velocity, proposed_angular_velocity, 
		current_linear_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
	steer = self.yaw_controller.get_steering(proposed_linear_velocity, 
						proposed_angular_velocity,
						current_linear_velocity)
	steer = self.pid_steer.step(steer, self.sample_time)
	speed_diff = sqrt(proposed_linear_velocity.x**2 + proposed_linear_velocity.y**2) - sqrt(current_linear_velocity.x**2 + current_linear_velocity.y**2)
	throttle = self.pid_speed(speed_diff, self.sample_time)
	if throttle < 0.0:
		brake = throttle
		throttle = 0
	else:
		brake = 0

	if not dbw_enabled:
		self.pid_speed.reset()
		self.pid_steer.reset()

        # Return throttle, brake, steer
        return 1., 0., 0.
