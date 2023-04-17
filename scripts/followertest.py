#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, Pose, TransformStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos, atan2
from enum import Enum
import tf2_ros
import tf


class Follower:
    def __init__(self):

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
        rospy.init_node('follower')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.pub_gotopose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.map_frame = rospy.get_param("~map_frame", 'map')
        self.robot_frame = rospy.get_param("~robot_frame", '/base_link')
        print("setup done")

    def process_scan(self, msg):
        num_scans = len(msg.ranges)
        scans_per_region = num_scans // 8
        ranges = [(index, r) for index, r in enumerate(msg.ranges)]
        self.regions = {
            "front": ranges[0 : scans_per_region - 1] + ranges[scans_per_region * 7 : -1],
            "left": ranges[scans_per_region: scans_per_region * 3 - 1],
            "back": ranges[scans_per_region * 3 : scans_per_region * 5 - 1],
            "right": ranges[scans_per_region * 5 : scans_per_region * 7 - 1]
        }

        self.increment = msg.angle_increment
        self.angle_min = msg.angle_min

    def process_odom(self, msg):
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_heading_radians = yaw
        self.position = msg.pose.pose.position


    def laser_target_vector(self, index, dist):
        print(f" angle min{self.angle_min}")
        dist_angle = self.angle_min + (index * self.increment)
        print(f"angle = {dist_angle}")
        x = dist * cos(dist_angle)
        y = dist * sin(dist_angle)

        return (x, y)

    def face_wall(self, index, dist):
        target_x, target_y = self.laser_target_vector(index, dist)
        target_heading = atan2(target_y - self.position.y, target_x - self.position.x)

        heading_diff = target_heading - self.current_heading_radians

        print("turning to face wall")
        while heading_diff > 0.1:
            self.make_move(0, min(0.5, max(-0.5, heading_diff)))

        self.make_move(0, 0)
        print("facing wall")

    def move_base_to_wall(self):
        while not self.regions['right']:
            continue
        closestIndex, closestRange = min(self.regions["front"], key= lambda x: x[1])
        print(f"{(closestIndex, closestRange)}")
        x, y = self.laser_target_vector(closestIndex, closestRange)

        print(f"target: {x}, {y}")

        listener = tf.TransformListener()

        laser_pose = PoseStamped()
        laser_pose.header.frame_id = "map"
        laser_pose.pose.position.x = 5.0
        laser_pose.pose.position.y = 2.0
        laser_pose.pose.position.z = 0.0
        laser_pose.pose.orientation.x = 0.0
        laser_pose.pose.orientation.y = 0.0
        laser_pose.pose.orientation.z = 0.0
        laser_pose.pose.orientation.w = 1.0

        # while not rospy.is_shutdown():
        #     try:
        #         listener.lookupTransform('map', "laser", rospy.Time(0))
        #         break
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #         print("EXCEPT. WAITING")
        #         # print exception below 
        #         print(e)
        #         continue

        # # Transform the pose from laser to map frame
        # transposed = listener.transformPose('map', laser_pose)
        # print("Pose transformed to: \n" + str(transposed.pose))

        # target_pose = transposed

        rate = rospy.Rate(1)
        rate.sleep()
        print("goal sent")
        self.publish_marker(laser_pose.pose)
        self.pub_gotopose.publish(laser_pose)

    def publish_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        point = Point()
        point.x = pose.orientation.x
        point.y = pose.orientation.y
        point.z = 0.0
        marker.points.append(point)

        self.marker_pub.publish(marker)   


if __name__ == '__main__':
    try:
        follower = Follower()
        follower.move_base_to_wall()
        rospy.spin()
        # follower.run()
    except rospy.ROSInterruptException:
        #end
        pass
