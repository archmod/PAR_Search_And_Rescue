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
