from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.pid_steer = PID(0.1, 0.0, 0.6)
        self.pid_steer.reset()
        min_speed = 1
        self.yawc = YawController(kwargs['wheel_base'], kwargs['steer_ratio'], min_speed,
        kwargs['max_lat_accel'], kwargs['max_steer_angle'])

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #prop_lin_v, prop_ang_v, curr_lin_v, dbw_enabled, time = args
        prop_lin_v, prop_ang_v, curr_lin_v, dbw_enabled = args
        time = rospy.get_rostime()
        rospy.loginfo('ctime, ctime_sec = %s, %s', time, time.to_sec())
        for (key, val) in kwargs.items():
            rospy.loginfo('params: %s = %s', key, val)

        prop_lin_vx, prop_lin_vy, prop_lin_vz = prop_lin_v
        curr_lin_vx, curr_lin_vy, curr_lin_vz = curr_lin_v
        prop_ang_vx, prop_ang_vy, prop_ang_vz = prop_ang_v

        steer = self.yawc.get_steering(prop_lin_vx, prop_ang_vz, curr_lin_vx)
        delta_steer = steer - prop_ang_vz
        #new_steer = self.pid_steer.step(delta_steer, 0.02)
        new_steer = self.pid_steer.step(steer, 0.02)
        rospy.loginfo('steer, delta_steer, new_steer = %s, %s, %s', steer, delta_steer, new_steer)
        return 1.0, 0.0, new_steer
        return 1., 0., 0.
