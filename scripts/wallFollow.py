#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Global variable to store the minimum distance to a wall
min_distance = 0.5

def laser_callback(scan):
    global min_distance
    ranges = scan.ranges
    # Check if there is an obstacle closer than the minimum distance
    first_10_min = min(ranges[:10])
    last_10_min = min(ranges[-10:])
    threshold = 0.2

    if first_10_min < threshold or last_10_min < threshold:
        reverse()
    else:
        turnRight()

def reverse():
    global pub
    # Create a Twist message with zero velocities
    reverse_twist = Twist()
    reverse_twist.linear.x = -0.2
    reverse_twist.angular.z = 0.0
    # Publish the message to stop the robot
    pub.publish(reverse_twist)

def turnRight():
    global pub
    # Create a Twist message with zero velocities
    turn_twist = Twist()
    turn_twist.linear.x = 0.0
    turn_twist.angular.z = -0.1
    # Publish the message to stop the robot
    pub.publish(turn_twist)

def stop_robot():
    global pub
    # Create a Twist message with zero velocities
    stop_twist = Twist()
    stop_twist.linear.x = 0.0
    stop_twist.angular.z = 0.0
    # Publish the message to stop the robot
    pub.publish(stop_twist)

def main():
    global pub

    rospy.init_node('wall_follow')
    
    # Subscribe to the laser scan topic
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    # Publish the velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
