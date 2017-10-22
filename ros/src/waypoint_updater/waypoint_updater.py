#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/current_pose', PoseStamped, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.wpts = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.loginfo('position = x:%s, y:%s, z:%s', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rospy.loginfo('orientation = x:%s, y:%s, z:%s, w:%s', msg.pose.orientation.x, 
                msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        if self.wpts is not None:
            rospy.loginfo('array size = %s', len(self.wpts.waypoints))
            num_forward_waypoints = 0
            forward_waypoints = []
            for wp in self.wpts.waypoints:
                if num_forward_waypoints == LOOKAHEAD_WPS:
                    break
                if wp.pose.pose.position.x > msg.pose.position.x:
                    num_forward_waypoints += 1
                    forward_waypoints.append(wp)

            wp0 = forward_waypoints[0]
            wp1 = forward_waypoints[-1]
            rospy.loginfo('wp position 0 = x:%s, y:%s, z:%s', wp0.pose.pose.position.x, wp0.pose.pose.position.y, wp0.pose.pose.position.z)
            rospy.loginfo('wp position -1 = x:%s, y:%s, z:%s', wp1.pose.pose.position.x, wp1.pose.pose.position.y, wp1.pose.pose.position.z)

        # publish the future waypoints
        
        fwrd_wpts = Lane()
        fwrd_wpts.header.stamp = rospy.Time.now()
        fwrd_wpts.header.frame_id = "final_wps"
        fwrd_wpts.waypoints = forward_waypoints

        self.final_waypoints_pub.publish(fwrd_wpts)

    def waypoints_cb(self, waypoints):
        if self.wpts is None:
            self.wpts = waypoints
            rospy.loginfo('array size = %s', len(self.wpts.waypoints))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
