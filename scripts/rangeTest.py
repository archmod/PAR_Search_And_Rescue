#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def process_scan(scan_msg):
    print("Scan length: " ,len(scan_msg.ranges))
    print("test")
    # print("Scan length: " ,scan_msg.ranges.size)


def rotate_rosbot():
    rospy.Subscriber('/scan', LaserScan, process_scan)
    rospy.init_node('rotate_rosbot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    print("test2")

    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    # twist.angular.z = 1.0  # Adjust this value to control the rotation speed

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        rotate_rosbot()
    except rospy.ROSInterruptException:
        pass
