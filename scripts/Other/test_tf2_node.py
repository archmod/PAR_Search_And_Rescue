import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv_bridge import CvBridge

class Tf2Node:
    def __init__(self):
        print("Running tf2 Node")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)
        self.image_to_world_coordinates()


    def image_to_world_coordinates(self, u, v, depth_image, camera_info, target_frame="map"):
        # Convert image coordinates to camera frame coordinates
        bridge = CvBridge()
        depth_cv = bridge.imgmsg_to_cv2(depth_image, desired_encoding='32FC1')
        depth = depth_cv[v, u]

        K = camera_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        # Create PointStamped message
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = camera_info.header.frame_id
        point_stamped.point.x = x
        point_stamped.point.y = y
        point_stamped.point.z = z

        # Transform point to world frame using tf2
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform = tf_buffer.lookup_transform(target_frame, point_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return transformed_point_stamped.point.x, transformed_point_stamped.point.y, transformed_point_stamped.point.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform object position to world frame")
            return None, None, None


if __name__ == '__main__':
    try:
        tf2Node = Tf2Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
