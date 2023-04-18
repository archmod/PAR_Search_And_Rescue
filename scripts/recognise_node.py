import rospy
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from enum import Enum
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray


class Recognition_Image(Enum):
    UNKNOWN = 0
    EXPLOSIVE = 1
    FLAMMABLE_GAS = 2
    NONFLAMMABLE_GAS = 3
    DANGEROUS_WHEN_WET = 4
    FLAMMABLE_SOLID = 5
    SPONTANEOUSLY_COMBUSTIBLE = 6
    OXIDIZER = 7
    ORGANIC_PEROXIDE = 8
    INHALATION_HAZARD = 9
    POISON = 10
    RADIOACTIVE = 11
    CORROSIVE = 12
    START = 99

CONFIDENCE_THRESHOLD = 0.5

class HazardDetection:
    def __init__(self):
        print("IM RUNNING BOSS")
        rospy.init_node('hazard_detection_node', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)

        # self.hazard_marker_pub = rospy.Publisher('/hazards', Marker, queue_size=10)
        self.hazard_marker_pub = rospy.Publisher('/hazards', MarkerArray, queue_size=10)
        self.hazard_markers = []

        self.start_marker_pub = rospy.Publisher('/startMarker', Int32, queue_size=10)
        self.lidar_scan = None
        rospy.Subscriber('/scan', LaserScan, self.lidar_scan_callback)
        self.fixed_marker_pub = rospy.Publisher('/fixed_marker', Marker, queue_size=1)

        self.mX = 0
        self.mY = 0
        self.published = False
        rospy.spin()

    def publish_data(self):
        while not rospy.is_shutdown():
            if self.published:
                print("printing hazard markers")
                # self.publish_fixed_marker(self.mX, self.mY)
                self.hazard_marker_pub.publish(self.hazard_markers)
                print(self.hazard_markers)
    
    def lidar_scan_callback(self, msg):
        self.lidar_scan = msg

    
    def transform_point_to_map_frame(self, point_stamped):
        try:
            target_frame = "map"
            source_frame = point_stamped.header.frame_id
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, point_stamped.header.stamp, rospy.Duration(1.0))

            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return transformed_point_stamped.point.x, transformed_point_stamped.point.y, transformed_point_stamped.point.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform object position to map frame")
            return None, None, None
            
    
    def is_hazard_present(self, x, y, hazard_id, threshold=0.1):
        for marker in self.hazard_markers:
            distance = math.sqrt((marker.pose.position.x - x)**2 + (marker.pose.position.y - y)**2)
            if distance < threshold and marker.id == hazard_id:
                return True
        return False

        
    def callback(self, msg):
        if self.lidar_scan is None:
            return
        print("Callback hit")

        for i in range(0, len(msg.objects.data), 12):
            object_id = int(msg.objects.data[i])
            confidence = msg.objects.data[i + 1]
            image_x = msg.objects.data[i + 7]
            image_y = msg.objects.data[i + 6]

            recognized_image = Recognition_Image(object_id)
            print(f"Recognized {recognized_image.name} (ID: {object_id}) with Confidence: {confidence}")
            if object_id == Recognition_Image.START.value:
                startMessage = Int32()
                startMessage.data = 1
                self.start_marker_pub.publish(startMessage)
                print("Start Marker Found")

            elif confidence > CONFIDENCE_THRESHOLD and object_id != Recognition_Image.START.value:

                # Get the angle corresponding to the detected object's image_x coordinate
                angle_increment = self.lidar_scan.angle_increment
                angle_min = self.lidar_scan.angle_min
                image_angle = angle_min + image_x * angle_increment

                # Get the distance to the detected object using the LiDAR data
                lidar_index = int((image_angle - angle_min) / angle_increment)
                distance = self.lidar_scan.ranges[lidar_index]

                # Create a PointStamped message for the object position in the laser frame
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.lidar_scan.header.frame_id
                point_stamped.header.stamp = rospy.Time(0)
                point_stamped.point.x = distance * math.cos(image_angle)
                point_stamped.point.y = distance * math.sin(image_angle)
                point_stamped.point.z = 0.0

                # Transform object position to map frame
                map_x, map_y, _ = self.transform_point_to_map_frame(point_stamped)
                if map_x is not None and map_y is not None:
                    if not self.is_hazard_present(map_x, map_y, object_id):
                        print(f"Published hazard marker {recognized_image.name} (ID: {object_id}) at (x, y): {map_x}, {map_y}")
                        hazard_marker = self.create_hazard_marker(object_id, map_x, map_y, 0)
                        self.hazard_markers.append(hazard_marker)
                        marker_array = MarkerArray()
                        marker_array.markers = self.hazard_markers
                        self.hazard_marker_pub.publish(marker_array)
                        # self.publish_fixed_marker(map_x, map_y)
                        rospy.sleep(0.3)

    
    def create_hazard_marker(self, marker_id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.id = marker_id
        return marker

    def publish_fixed_marker(self,x, y):
        fixed_marker = Marker()
        fixed_marker.header.frame_id = "map"
        fixed_marker.type = Marker.CUBE
        fixed_marker.action = Marker.ADD
        fixed_marker.id = 0

        fixed_marker.pose.position.x = x
        fixed_marker.pose.position.y = y
        fixed_marker.pose.position.z = 0
        fixed_marker.pose.orientation.x = 0.0
        fixed_marker.pose.orientation.y = 0.0
        fixed_marker.pose.orientation.z = 0.0
        fixed_marker.pose.orientation.w = 1.0

        fixed_marker.scale.x = 0.3
        fixed_marker.scale.y = 0.3
        fixed_marker.scale.z = 0.3

        fixed_marker.color.r = 1.0
        fixed_marker.color.g = 1.0
        fixed_marker.color.b = 0.0
        fixed_marker.color.a = 1.0
        self.fixed_marker_pub.publish(fixed_marker)


if __name__ == '__main__':
    try:
        hazard_detection_node = HazardDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
