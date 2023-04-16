#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import pi
from enum import Enum
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
import tf2_ros
import tf2_geometry_msgs

printLogs = True

class Bot_state(Enum):
    IDLE = 0
    FIND_WALL = 1
    FOLLOW_wALL = 2
    TURN_LEFT = 3


class Follower:
    def __init__(self):
        rospy.init_node('follower')
        self.starting_pos_recorded = False
        self.start_image_detected = False

        self.start_time = rospy.Time.now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.start_marker_pub = rospy.Publisher('/starting_position_marker', Marker, queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.threshold_min = rospy.get_param("~threshold_min", 0.3)
        self.threshold_max = rospy.get_param("~threshold_max", 0.5)
        #defaults to rosbot 1 unit on the grid
        self.minWallDist = rospy.get_param("~minWallDistance", 0.3)
        self.bot_state = Bot_state.IDLE
        self.actions = {
            Bot_state.IDLE: self.do_nothing,
            Bot_state.FIND_WALL: self.find_wall,
            Bot_state.TURN_LEFT: self.turn_left
        }

        self.regions = {
            "front": [],
            "right": [],
            "back": [],
            "right": []
        }
        rospy.Subscriber('/startMarker', Int32, self.start_marker_callback)
        
    def process_scan(self, msg):
        num_scans = len(msg.ranges)
        scans_per_region = num_scans // 8
        ranges = list(msg.ranges)
        self.regions = {
            "front": ranges[0 : scans_per_region - 1] + ranges[scans_per_region * 7 : -1],
            "left": ranges[scans_per_region: scans_per_region * 3 - 1],
            "back": ranges[scans_per_region * 3 : scans_per_region * 5 - 1],
            "right": ranges[scans_per_region * 5 : scans_per_region * 7 - 1]
        }

        self.select_action()

    def run(self):
        while True:
            self.select_action()
            rospy.spin()
    
    def start_marker_callback(self, msg):
        if not self.start_image_detected:
            self.start_image_detected = True
            print("Received start marker. Starting to follow wall.")

    def select_action(self):
        timeDelta = rospy.Time.now() - self.start_time
        if self.start_image_detected or timeDelta.to_sec() > 5:
            print("Doing shit")
            if not self.regions['right']:
                if printLogs:
                    print("passing")
                pass
            # print(self.regions["right"])
            closestRight = min(min(self.regions["right"]), 15)
            if printLogs:
                print(closestRight)
            if min(self.regions['front']) < self.threshold_max:
                self.turn_left()
            elif closestRight > self.threshold_min and closestRight < self.threshold_max:
                self.follow_wall()
            elif closestRight < self.threshold_min:
                self.turn_left()
                self.follow_wall()
            else:
                self.find_wall()
        else:
            self.do_nothing()
        

        #if wall exists on right 
            #go to the wall
            #when close enough to the wall 3 - 1.5 from wall
                #go straight
                #if too close turn left
                #if too far away turn right
                #if wall in front and right turn left

    def do_nothing(self):
        print("Doing nothing")
        self.record_starting_pose()
        pass
    
    def find_wall(self):
        if printLogs:
            print("find wall")
        self.make_move(0.1, -0.3)

    def turn_left(self):
        if printLogs:
            print("turn left")
        self.make_move(0, 0.3)

    def turn_right(self):
        if printLogs:
            print("turn right")
        self.make_move(0, -0.3)
    
    def follow_wall(self):
        if printLogs:
            print("follow wall")
        self.make_move(0.1, 0)
    
    def make_move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)
    
    def record_starting_pose(self):
        try:
            print("Trying to get initial pose in map coordinates")
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            self.starting_pose = PoseStamped()
            self.starting_pose.header.frame_id = "map"
            self.starting_pose.header.stamp = rospy.Time.now()
            self.starting_pose.pose.position = transform.transform.translation
            self.starting_pose.pose.orientation = transform.transform.rotation
            print("Starting pose recorded:")
            self.publish_starting_pose_cube()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get initial pose in map coordinates")
            return None
        
    def publish_starting_pose_cube(self):
        if self.starting_pose is not None:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "starting_position"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = self.starting_pose.pose.position
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            self.start_marker_pub.publish(marker)
            self.starting_pos_recorded = True


if __name__ == '__main__':
    try:
        follower = Follower()
        rospy.spin()
        # follower.run()
    except rospy.ROSInterruptException:
        #end
        pass
