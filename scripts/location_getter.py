import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import tf
import json

class Location:
    def __init__(self):
        rospy.init_node('robot_location')
        self.robot_pose = PoseStamped()
        self.listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('/location_marker', Marker, queue_size=1)
        self.pose_publisher()

    def tf_callback(self, msg):
        if msg.header.frame_id == 'map' and msg.child_frame_id == 'base_link':
            # Convert transform to pose
            self.robot_pose.pose.position.x = msg.transform.translation.x
            self.robot_pose.pose.position.y = msg.transform.translation.y
            self.robot_pose.pose.position.z = msg.transform.translation.z
            self.robot_pose.pose.orientation.x = msg.transform.rotation.x
            self.robot_pose.pose.orientation.y = msg.transform.rotation.y
            self.robot_pose.pose.orientation.z = msg.transform.rotation.z
            self.robot_pose.pose.orientation.w = msg.transform.rotation.w
            self.robot_pose.header.stamp = msg.header.stamp
            self.robot_pose.header.frame_id = 'map'


    # Set up publisher for storing robot pose in a ROS parameter
    def pose_publisher(self):
        rate = rospy.Rate(100) # Update rate in Hz
        while not rospy.is_shutdown():
            try:
                # Lookup transform from map to base_link
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                # Convert transform to pose
                self.robot_pose.pose.position.x = trans[0]
                self.robot_pose.pose.position.y = trans[1]
                self.robot_pose.pose.position.z = trans[2]
                self.robot_pose.pose.orientation.x = rot[0]
                self.robot_pose.pose.orientation.y = rot[1]
                self.robot_pose.pose.orientation.z = rot[2]
                self.robot_pose.pose.orientation.w = rot[3]
                self.robot_pose.header.stamp = rospy.Time.now()
                self.robot_pose.header.frame_id = 'map'
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            # Convert PoseStamped message to JSON string
            pose_json = json.dumps({
                'position': {
                    'x': self.robot_pose.pose.position.x,
                    'y': self.robot_pose.pose.position.y,
                    'z': self.robot_pose.pose.position.z
                },
                'orientation': {
                    'x': self.robot_pose.pose.orientation.x,
                    'y': self.robot_pose.pose.orientation.y,
                    'z': self.robot_pose.pose.orientation.z,
                    'w': self.robot_pose.pose.orientation.w
                }
            })

            rospy.loginfo('Robot pose: %s', pose_json)
            self.marker_pub.publish(self.create_marker(self.robot_pose))


    def create_marker(self, pose):
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
        marker.color.g = 0.1
        marker.color.b = 0.0

        marker.lifetime = rospy.Duration()

        return marker

if __name__ == '__main__':
    try:
        location = Location()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

