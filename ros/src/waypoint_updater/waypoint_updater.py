#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

#my imports
from std_msgs.msg import Int32
import tf

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
SPEED=35 #speed in Kmh

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb,queue_size=1)#,queue_size add queue for simulator lag
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb,queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane,queue_size=1)#queue_size

        # TODO: Add other member variables you need below
        self.my_current_pose = None
        self.waypoints = None
        rospy.spin()#sleep until rospy.is_shutdown() is true

    def pose_cb(self, msg):
        # TODO: Implement
        self.my_current_pose = msg.pose
        self.compute_final_waypoints()
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints
        pass

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

    def closest_waypoint(self):
    # from CarND-Path-Planning-Project
        closestlen = 1000000
        closestwaypoint = 0
        eucdist = lambda a, b: math.sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y))
        for index, waypoint in enumerate(self.waypoints):
            dist = eucdist(self.my_current_pose.position, waypoint.pose.pose.position)
            if (dist < closestlen):
                closestlen = dist
                closestwaypoint = index
        return closestwaypoint

    def next_waypoint(self):
    # from CarND-Path-Planning-Project
        closestwaypoint=self.closest_waypoint()
        map_x=self.waypoints[closestwaypoint].pose.pose.position.x
        map_y=self.waypoints[closestwaypoint].pose.pose.position.y
        heading=math.atan2((map_y-self.my_current_pose.position.y),(map_x-self.my_current_pose.position.x))
        quaternion = (self.my_current_pose.orientation.x,self.my_current_pose.orientation.y,self.my_current_pose.orientation.z,self.my_current_pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(quaternion)
        theta=yaw[2]
        angle=math.fabs(theta-heading)
        if angle>(math.pi/4):
            closestwaypoint+=1
        return   closestwaypoint


    def compute_final_waypoints(self):

        if self.my_current_pose is not None and self.waypoints is not None:

                first_waypoint_ahead = self.next_waypoint()# from CarND-Path-Planning-Project
                #rospy.logwarn("first waypoint ahead %s", first_waypoint_ahead)
                waypointscurrentlyahead = self.waypoints[first_waypoint_ahead:first_waypoint_ahead + LOOKAHEAD_WPS]


                for i in range(len(waypointscurrentlyahead) - 1):
                    self.set_waypoint_velocity(waypointscurrentlyahead, i,SPEED*0.2777)#km/h to m/s

                wplane = Lane()
                wplane.header.frame_id = '/finalwaypoints'
                wplane.header.stamp =  rospy.Time.now()
                wplane.waypoints = waypointscurrentlyahead
                self.final_waypoints_pub.publish(wplane)#publish the next lookahead points



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
