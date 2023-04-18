#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos, atan2
from enum import Enum

class Bot_state(Enum):
    START = 0
    FIND_WALL = 1
    FOLLOW_wALL = 2
    TURN_LEFT = 3
    IDLE = 4
    FIRST_WALL = 5
    FOUND_WALL = 6


class Follower:
    def __init__(self):
        self.threshold_min = rospy.get_param("~threshold_min", 0.3)
        self.threshold_max = rospy.get_param("~threshold_max", 0.5)
        #defaults to rosbot 1 unit on the grid
        # self.minWallDist = rospy.get_param("~minWallDistance", 0.5)
        self.bot_state = Bot_state.START

        self.regions = {
            "front": [],
            "right": [],
            "back": [],
            "right": []
        }

        self.increment = 0
        self.angle_min = -3.1415927410125732
        self.current_heading_radians = None
        self.position = None
        self.prev_direct_right = 10
        rospy.init_node('follower')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # rospy.Subscriber('/odom', Odometry, self.process_odom)
        print("setup done")

    def process_scan(self, msg):
        print("message")
        num_scans = len(msg.ranges)
        scans_per_region = num_scans // 8
        ranges = [(index, r) for index, r in enumerate(msg.ranges)]
        self.regions = {
            "front": ranges[0 : scans_per_region - 1] + ranges[scans_per_region * 7 : -1],
            "left": ranges[scans_per_region: scans_per_region * 3 - 1],
            "back": ranges[scans_per_region * 3 : scans_per_region * 5 - 1],
            "right": ranges[scans_per_region * 5 : scans_per_region * 6 - 1]
        }
        self.direct_right_range = msg.ranges[len(msg.ranges)//4]
        self.right_range_diff = self.prev_direct_right - self.direct_right_range
        self.prev_direct_right = self.direct_right_range
        self.increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.select_action()

    def process_odom(self, msg):
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_heading_radians = yaw
        self.position = msg.pose.pose.position


    def run(self):
        while True:
            self.select_action()
            rospy.spin()

    def select_action(self):
        if not self.regions['right']:
            print("regions not loaded")
            pass
        _, closestRangeRight = min(self.regions["right"], key= lambda x: x[1])
        _, closestRangeFront = min(self.regions["front"], key= lambda x: x[1])

        
        if self.bot_state == Bot_state.START:
            print("START")
            self.start(closestRangeRight)

        elif self.bot_state == Bot_state.FIRST_WALL:
            self.first_wall(closestRangeFront)

        elif self.bot_state == Bot_state.FOUND_WALL:
            self.follow_wall(closestRangeRight, closestRangeFront)
    

    def start(self, closestRight):
        if closestRight < self.threshold_min:
            self.go_straight()
            print("FOUND WALL")
            self.bot_state == Bot_state.FOUND_WALL
        else:
            self.right_turn_90()
            self.go_straight()
            print("FIRST WALL")
            self.bot_state = Bot_state.FIRST_WALL

    def first_wall(self, closestFront):
        print(closestFront)
        if closestFront < self.threshold_max:
            self.turn_left()
            print("FOUND WALL")
            self.bot_state = Bot_state.FOUND_WALL
        else:
            self.go_straight()

    def follow_wall(self, closestRight, closestFront):
        print(closestRight)
        print(self.direct_right_range)
        print(self.right_range_diff)
        
        if closestFront < self.threshold_min:
            self.turn_left()
        elif closestRight > self.threshold_max:
            self.back_to_wall()
        elif closestRight > self.threshold_min and closestRight < self.threshold_max:
            self.go_straight()
        elif closestRight < self.threshold_min:
            self.turn_left()
            self.go_straight()
        elif self.right_range_diff < -(self.threshold_max):
            print("OPENING")
            self.right_turn_90()
            self.go_straight()
        else:
            self.back_to_wall()

    def do_nothing(self):
        print("doing nothing")

    
    def back_to_wall(self):
        print("back to wall")
        self.make_move(0.1, -0.3)

    def turn_left(self):
        print("turn left")
        self.make_move(0, 0.3)

    def right_turn_90(self):
        duration = rospy.Duration.from_sec(3.0)  # seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < duration:
            self.make_move(0, -0.5)
            rospy.sleep(0.1)

        self.make_move(0,0)

    def turn_right(self):
        print("turn right")
        self.make_move(0, -0.3)
    
    def go_straight(self):
        print("go straight")
        self.make_move(0.1, 0)
    
    def make_move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

        


if __name__ == '__main__':
    try:
        follower = Follower()
        print("blow me")
        rospy.spin()
        # follower.run()
    except rospy.ROSInterruptException:
        #end
        pass
