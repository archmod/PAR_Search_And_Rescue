import math
from matplotlib import mathtext
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathRecorderNode:
    def __init__(self):
        rospy.init_node('path_recorder_node')
        self.totalDistance = 0
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.prev_pose = None

        self.record_path()

    def record_path(self):
        rate = rospy.Rate(1)  # Set the rate to 1 Hz

        while not rospy.is_shutdown():
            try:
                # Get the latest transform from map to base_link
                transform_stamped = self.buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(5.0))

                # Convert the transform to a PoseStamped message
                pose_stamped = PoseStamped()
                pose_stamped.header = transform_stamped.header
                pose_stamped.pose.position.x = transform_stamped.transform.translation.x
                pose_stamped.pose.position.y = transform_stamped.transform.translation.y
                pose_stamped.pose.position.z = transform_stamped.transform.translation.z
                pose_stamped.pose.orientation = transform_stamped.transform.rotation

                # Add the pose to the path
                self.path.poses.append(pose_stamped)

                # Calculate the distance traveled
                # if self.prev_pose is not None:
                #     distance = self.distance_between_poses(self.prev_pose, pose_stamped)
                #     self.total_distance += distance
                #     rospy.loginfo("Total distance travelled: %f", self.total_distance)

                self.prev_pose = pose_stamped

                # Publish the path
                self.path_pub.publish(self.path)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Waiting for transform")
            rate.sleep()

    def distance_between_poses(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

if __name__ == '__main__':
    try:
        path_recorder_node = PathRecorderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
