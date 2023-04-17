# import rospy
# from geometry_msgs.msg import PoseStamped, Twist, Pose, TransformStamped, Point
# from visualization_msgs.msg import Marker
# from nav_msgs.msg import Path
# import tf
# import json
# import math

# class PathNode:
#     def __init__(self):
#         rospy.init_node('path_node')
#         self.robot_pose = PoseStamped()
#         self.listener = tf.TransformListener()
#         self.all_recorded_poses = []

#         self.path_pub = rospy.Publisher('path_taken', Path, queue_size=9999)
#         self.marker_pub = rospy.Publisher('/travelled_marker', Marker, queue_size=9999)
#         self.start_marker_pub = rospy.Publisher('/start_travelled_marker', Marker, queue_size=9999)
#         self.start_marker_is_not_set = True
#         self.total_distance = 0.0
#         rospy.sleep(1) # Important ! Allows Detection of True Values
#         self.pose_publisher()

#     def pose_publisher(self):
#         prev_pose = None

#         while not rospy.is_shutdown():
#             try:
#                 # Lookup transform from map to base_link
#                 (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
#                 # Convert transform to pose
#                 robot_pose = PoseStamped()
#                 robot_pose.pose.position.x = trans[0]
#                 robot_pose.pose.position.y = trans[1]
#                 robot_pose.pose.position.z = trans[2]
#                 robot_pose.pose.orientation.x = rot[0]
#                 robot_pose.pose.orientation.y = rot[1]
#                 robot_pose.pose.orientation.z = rot[2]
#                 robot_pose.pose.orientation.w = rot[3]
#                 robot_pose.header.stamp = rospy.Time.now()
#                 robot_pose.header.frame_id = 'map'
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#                 print("except path_node:", e)
#                 continue

#             if prev_pose is not None:
#                 self.total_distance += self.distance_between_poses(prev_pose, robot_pose)
#                 rospy.loginfo("Total distance travelled: %f", self.total_distance)

#             prev_pose = robot_pose

#             self.all_recorded_poses.append(robot_pose)
#             self.create_updated_path()

#             if self.start_marker_is_not_set == True:
#                 rospy.loginfo("start marker found")
#                 self.start_marker_pub.publish(self.create_marker(robot_pose))
#                 self.start_marker_is_not_set = False
#             else:
#                 self.marker_pub.publish(self.create_marker(robot_pose))

#             # Add Delay !
#             rospy.sleep(5)


#     def distance_between_poses(self, pose1, pose2):
#         dx = pose1.pose.position.x - pose2.pose.position.x
#         dy = pose1.pose.position.y - pose2.pose.position.y
#         return math.sqrt(dx * dx + dy * dy)
    
#     def create_marker(self, pose):
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.type = marker.POINTS
#         marker.action = marker.ADD

#         marker.pose = pose.pose
#         point = Point()
#         point.x = pose.pose.position.x
#         point.y = pose.pose.position.y
#         point.z = 0
#         marker.points.append(point)

#         marker.scale.x = 0.1
#         marker.scale.y = 0.1
#         marker.scale.z = 0.1

#         # Green --
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 0.1
#         marker.color.b = 0.0

#         marker.lifetime = rospy.Duration(0)  # Set the marker lifetime to 0, making it permanent

#         return marker

#     def create_updated_path(self):
#         path = Path()
#         path.header.frame_id = 'map'
#         path.header.stamp = rospy.Time.now()
#         path.poses = self.all_recorded_poses
#         self.path_pub.publish(path)

# if __name__ == '__main__':
#     try:
#         path_node = PathNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker

class PathNode:
    def __init__(self):
        rospy.init_node('path_node')

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.start_marker_pub = rospy.Publisher('/start_marker', Marker, queue_size=1)
        self.path_marker_pub = rospy.Publisher('/path', Marker, queue_size=10)
        self.publish_start_marker()

    def publish_start_marker(self):
        while not rospy.is_shutdown():
            try:
                # Get the latest transform from map to base_link
                transform_stamped = self.buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(5.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Waiting for transform")
                continue

        start_pose = PoseStamped()
        start_pose.header = transform_stamped.header
        start_pose.pose.position.x = transform_stamped.transform.translation.x
        start_pose.pose.position.y = transform_stamped.transform.translation.y
        start_pose.pose.position.z = transform_stamped.transform.translation.z
        start_pose.pose.orientation = transform_stamped.transform.rotation

        start_marker = self.create_start_marker(start_pose)

        while not rospy.is_shutdown():
            self.start_marker_pub.publish(start_marker)
            rospy.sleep(1)


    def create_start_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD

        marker.pose = pose.pose

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Green
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

if __name__ == '__main__':
    try:
        path_node = PathNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
