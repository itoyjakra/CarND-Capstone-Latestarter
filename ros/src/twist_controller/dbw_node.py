#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_params = {
                'vehicle_mass': rospy.get_param('~vehicle_mass', 1736.35),
                'fuel_capacity': rospy.get_param('~fuel_capacity', 13.5),
                'brake_deadband': rospy.get_param('~brake_deadband', .1),
                'decel_limit': rospy.get_param('~decel_limit', -5),
                'accel_limit': rospy.get_param('~accel_limit', 1.),
                'wheel_radius': rospy.get_param('~wheel_radius', 0.2413),
                'wheel_base': rospy.get_param('~wheel_base', 2.8498),
                'steer_ratio': rospy.get_param('~steer_ratio', 14.8),
                'max_lat_accel': rospy.get_param('~max_lat_accel', 3.),
                'max_steer_angle': rospy.get_param('~max_steer_angle', 8.)
                }



        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        self.controller = Controller(**self.vehicle_params)

        # TODO: Subscribe to all the topics you need to
        self.dbw_enabled = False;
        self.curr_time = None
        self.prop_lin_v = [1.0, 2.0, 3.0]
        self.prop_ang_v = [4.0, 5.0, 6.0]
        self.curr_lin_v = [7.0, 8.0, 9.0]

        rospy.Subscriber('/twist_cmd', TwistStamped, self.propvel_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.currvel_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        rospy.loginfo('dbw enabled = %s', self.dbw_enabled)

        while not rospy.is_shutdown():
            rospy.loginfo('curr_lin_v = x:%s, y:%s, z:%s', self.curr_lin_v[0],self.curr_lin_v[1],self.curr_lin_v[2])
            rospy.loginfo('prop_lin_v = x:%s, y:%s, z:%s', self.prop_lin_v[0],self.prop_lin_v[1],self.prop_lin_v[2])
            rospy.loginfo('prop_ang_v = x:%s, y:%s, z:%s', self.prop_ang_v[0],self.prop_ang_v[1],self.prop_ang_v[2])
            rospy.loginfo('time = %s', self.curr_time)
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            throttle, brake, steering = self.controller.control(self.prop_lin_v,
                                                                self.prop_ang_v,
                                                                self.curr_lin_v,
                                                                self.dbw_enabled,
                                                                #self.curr_time,
                                                                **self.vehicle_params)
            #throttle = 0.4; brake = 0.0; steer = 0.0
            if self.dbw_enabled:
               self.publish(throttle, brake, steering)
               rospy.loginfo('th, br, st = %s, %s, %s', throttle, brake, steering)
            rate.sleep()

    def enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo('dbw enabled = %s', self.dbw_enabled)

    def currvel_cb(self, msg):
        self.curr_lin_v = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def propvel_cb(self, msg):
        self.prop_lin_v = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.prop_ang_v = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        self.curr_time = msg.header.stamp.secs

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
