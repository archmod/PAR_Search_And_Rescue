import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import tf
import json

class Location:
    def __init__(self):
        rospy.init_node('robot_location_getter')
        self.robot_pose = PoseStamped()
        self.listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('/travelled_marker', Marker, queue_size=1)
        self.start_marker_pub = rospy.Publisher('/start_travelled_marker', Marker, queue_size=1)
        self.start_marker_is_not_set = True
        self.pose_publisher()

    def pose_publisher(self):
        rospy.sleep(1) # Important ! Allows Detection of True Values

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

            if self.start_marker_is_not_set == True:
                rospy.loginfo("start marker found")
                self.start_marker_pub.publish(self.create_marker(self.robot_pose))
                self.start_marker_is_not_set = False
                rospy.loginfo('Starting pose: %s', pose_json)
            else:
                self.marker_pub.publish(self.create_marker(self.robot_pose))
                rospy.loginfo('Robot pose: %s', pose_json)


            # Add Delay !
            rospy.sleep(5)

    def create_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD

        marker.pose = pose.pose

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Green -- 
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

