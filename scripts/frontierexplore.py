#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker
import tf
import math

class FrontierExplore:
    def __init__(self):
        rospy.init_node('explorer')
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.frontier = []
        rospy.Subscriber('/generate/frontier', PoseArray, self.process_frontier)
        self.listener = tf.TransformListener()

    def process_frontier(self, msg):
        self.frontier = msg.poses
        print(f"frontier len {len(self.frontier)}")

    def explore_frontier(self):
        rate = rospy.Rate(1)
        while not self.frontier:
            print("waiting for frontier")
            rate.sleep()
        
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        ((x, y, _), rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        next_point = self.closest_point(self.frontier, (x, y))

        pose_s = PoseStamped()
        pose_s.header.frame_id = "map"
        pose_s.pose = next_point

        self.publish_marker(next_point)

        rate.sleep()
        print("goal sent")
        print(f"moving to {pose_s}")
        self.pub.publish(pose_s)

    def closest_point(self, frontier, pos):
        closest = (frontier[0], 10000)
        for point in frontier:
            dist = math.dist((point.position.x, point.position.y), pos)
            if dist < closest[1]:
                closest = (point, dist)

        return closest[0]

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
        explorer = FrontierExplore()
        explorer.explore_frontier()
        rospy.spin()
        # follower.run()
    except rospy.ROSInterruptException:
        #end
        pass


        

