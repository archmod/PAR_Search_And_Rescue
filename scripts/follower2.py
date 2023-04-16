#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi
from enum import Enum

printLogs = False

class Bot_state(Enum):
    IDLE = 0
    FIND_WALL = 1
    FOLLOW_wALL = 2
    TURN_LEFT = 3


class Follower:
    def __init__(self):
        rospy.init_node('follower')
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

        self.start_image_detected = False

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

    def select_action(self):
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
        

        #if wall exists on right 
            #go to the wall
            #when close enough to the wall 3 - 1.5 from wall
                #go straight
                #if too close turn left
                #if too far away turn right
                #if wall in front and right turn left

    def do_nothing(self):
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

if __name__ == '__main__':
    try:
        follower = Follower()
        rospy.spin()
        # follower.run()
    except rospy.ROSInterruptException:
        #end
        pass
