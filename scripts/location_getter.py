import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import tf
import json

class Location:
    def __init__(self):
        rospy.init_node('robot_location_getter')
        self.robot_pose = PoseStamped()
        self.listener = tf.TransformListener()
        self.all_recorded_poses = []

        self.path_pub = rospy.Publisher('path_taken', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/travelled_marker', Marker, queue_size=1)
        self.start_marker_pub = rospy.Publisher('/start_travelled_marker', Marker, queue_size=1)
        self.start_marker_is_not_set = True
        rospy.sleep(1) # Important ! Allows Detection of True Values
        self.pose_publisher()

    def pose_publisher(self):

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

            # For Logging Purposes Only
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

            self.all_recorded_poses.append(self.robot_pose)
            self.create_updated_path()

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

        marker.lifetime = rospy.Duration(42600)

        return marker
    
    def create_updated_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        path.poses = self.all_recorded_poses
        self.path_pub.publish(path)

if __name__ == '__main__':
    try:
        location = Location()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

