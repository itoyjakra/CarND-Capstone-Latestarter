from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.pid_throttle = PID(4.0, 0.0, 0.0)
        min_speed = 1
        self.yawc = YawController(kwargs['wheel_base'], kwargs['steer_ratio'], min_speed,
        kwargs['max_lat_accel'], kwargs['max_steer_angle'])

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        prop_lin_v, prop_ang_v, curr_lin_v, dbw_enabled = args
        if not dbw_enabled:
            self.pid_throttle.reset()
            rospy.loginfo('resetting throttle pid')
            return 0.0, 0.0, 0.0
        else:
            time = rospy.get_rostime()
            rospy.loginfo('ctime, ctime_sec = %s, %s', time, time.to_sec())
            for (key, val) in kwargs.items():
                rospy.loginfo('params: %s = %s', key, val)

            prop_lin_vx, prop_lin_vy, prop_lin_vz = prop_lin_v
            curr_lin_vx, curr_lin_vy, curr_lin_vz = curr_lin_v
            prop_ang_vx, prop_ang_vy, prop_ang_vz = prop_ang_v

            error_speed = prop_lin_vx - curr_lin_vx
            throttle = self.pid_throttle.step(error_speed, 0.02)

            steer = self.yawc.get_steering(prop_lin_vx, prop_ang_vz, curr_lin_vx)
            rospy.loginfo('error_speed, throttle, steer_val = %s, %s, %s', error_speed, throttle, steer)
            return throttle, 0.0, steer
