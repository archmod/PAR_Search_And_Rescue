import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathRecorderNode:
    def __init__(self):
        rospy.init_node('path_recorder_node')

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)

        self.path = Path()
        self.path.header.frame_id = 'map'

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

                # Publish the path
                self.path_pub.publish(self.path)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Waiting for transform")
            rate.sleep()

if __name__ == '__main__':
    try:
        path_recorder_node = PathRecorderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
