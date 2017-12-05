#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from itertools import islice, cycle

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

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.current_pos = None
        self.last_wp_ind = None
        self.max_speed_meters_per_sec=0

        rospy.spin()

    def pose_cb(self, PoseStamped):
        # TODO: Implement    
        self.current_pos = PoseStamped.pose
        #rospy.loginfo("WP_Updater: Car position updated to %s", self.current_pose)
        self.gen_next_waypoints()
                 

    def waypoints_cb(self, Lane):
        # TODO: Implement
        if self.waypoints is None:
            self.waypoints = Lane.waypoints
            if len(Lane.waypoints) > 0:
                self.max_speed_meters_per_sec = Lane.waypoints[0].twist.twist.linear.x
            rospy.loginfo('WP_Updater: initial waypoints recieved')
            #self.gen_next_waypoints()
        

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
    
    def gen_next_waypoints(self):
        #self.waypoints and  self.current_pose will be null if simulator is not running
        #Check if it is none and return if None
        if self.waypoints is None or self.current_pos is None :
           return
        carx = self.current_pos.position.x
        cary = self.current_pos.position.y
        
        rospy.loginfo("WP_Updater: Searching closest waypoint at position %f, %f", carx, cary)
        
        min_distance = float("inf")
        min_dist_loc = None
        start_index = 0
        num_pts = len(self.waypoints)
        num_inds = num_pts
        #rospy.logdebug("WP_Updater: start_index and end_index %f, %f", start_index, end_index)
        if self.last_wp_ind is not None:
            start_index = self.last_wp_ind
            num_inds=30 # 30 ahead should be enough            
        for i in range(0, num_inds):
            ind = (start_index+i)%num_pts
            waypoint = self.waypoints[ind]
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            # dist between car and waypoint
            dist = math.sqrt((carx -waypoint_x)**2  + (cary -waypoint_y)**2)
            
            # check if this distance is minimum distance calculated so far
            if dist < min_distance:
                min_distance = dist
                min_dist_loc = ind 
        
        closest_wp = self.waypoints[min_dist_loc]
        closest_wp_pos = closest_wp.pose.pose.position
        self.last_wp_ind = min_dist_loc
        rospy.logdebug("WP_Updater: Closest waypoint- idx:%d x:%f y:%f", min_dist_loc, closest_wp_pos.x, closest_wp_pos.y);
        #cycle(waypoints) is needed as world is cyclical and if we find nearest point at top end of waypoints array we may not have
        #enough LOOKAHEAD_WPS waypoints ahead of us 
        next_waypoints = list(islice(cycle(self.waypoints),min_dist_loc, min_dist_loc+LOOKAHEAD_WPS))
        rospy.logdebug("WP_Updater: length of next_waypoints:%f",len(next_waypoints))
        
        for i in range(len(next_waypoints)):
            self.set_waypoint_velocity(next_waypoints, i, self.max_speed_meters_per_sec)
        
        lane = Lane()
        lane.waypoints = next_waypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)
            
        
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')