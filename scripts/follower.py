#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.follow_closest_wall)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.speed = rospy.get_param("~speed", 0.5)
        self.turnRate = rospy.get_param("~turnRate", 0.8)
        #defaults to rosbot 1 unit on the grid
        self.minWallDist = rospy.get_param("~minWallDistance", 0.5)

    def follow_closest_wall(self, msg):
        ranges = msg.ranges
        twist = Twist()
        # find the ranges to the left side of the robot
        # doesn't work
        leftPoints = []
        print(len(msg.ranges))
        for i, r in enumerate(ranges):
            angleRad = msg.angle_min + i * msg.angle_increment
            if angleRad >= -pi / 2 and angleRad <= pi / 2:
                leftPoints.append(r)

        minIndex, minRange = min(enumerate(ranges), key= lambda r: r[1])

        rotation = 0.8 * (2 * minIndex / (len(ranges) - 1)) - 1
        otherRotation = 0.8 * (minIndex - len(ranges) / 2) / (len(ranges) / 2)

        print(f"({msg.angle_max} - {msg.angle_min}) / {msg.angle_increment} = {(msg.angle_max - msg.angle_max) / msg.angle_increment}")

        if minRange > self.minWallDist:
            #if outside min distance drive towards the wall
            twist.linear.x = self.speed
            twist.angular.z = rotation
        else:
            #if too close to wall just spin
            twist.linear.x = 0
            twist.angular.z = rotation

        self.pub.publish(twist)


if __name__ == '__main__':
    try:
        wf = WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        #end
        pass
