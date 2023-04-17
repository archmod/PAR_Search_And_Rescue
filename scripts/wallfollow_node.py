#!/usr/bin/env python

import rospy
import math
import actionlib
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class WallFollower:
    def __init__(self):
        rospy.init_node('rosbot_wall_follower', anonymous=True)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

        self.ranges = []
        self.fixed_distance = 0.1  # Fixed distance to follow the wall

        self.wall_follower()

    def laser_scan_callback(self, msg):
        self.ranges = msg.ranges

    def wall_follower(self):
        while not rospy.is_shutdown():
            if self.ranges:
                # Find the closest point on the left side of the robot
                left_ranges = self.ranges[0:180]  
                min_dist = min(left_ranges)
                min_dist_index = left_ranges.index(min_dist)
                angle = min_dist_index * 0.349066  # Convert to radians

                # Calculate the goal position in the base_link frame
                goal_x = (min_dist + self.fixed_distance) * math.cos(angle)
                goal_y = (min_dist + self.fixed_distance) * math.sin(angle)

                # Send the goal
                self.send_goal(goal_x, goal_y, angle)

            rospy.sleep(0.1)

    def send_goal(self, goal_x, goal_y, angle):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y

        quaternion = quaternion_from_euler(0, 0, angle)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

if __name__ == '__main__':
    try:
        WallFollower()
    except rospy.ROSInterruptException:
        pass
