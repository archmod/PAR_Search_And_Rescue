import rospy
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from enum import Enum
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

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

signage_id = {
    # 'Unknown' Included in the Assignment Specs - Potential New Sign shown on Day?
    0: 'Unknown',
    1: 'Explosive',
    2: 'Flammable Gas',
    3: 'Non-Flammable Gas',
    4: 'Dangerous When Wet',
    5: 'Flammable Solid',
    6: 'Spontaneously Combustible',
    7: 'Oxidizer',
    8: 'Organic Peroxide',
    9: 'Inhalation Hazard',
    10: 'Poison',
    11: 'Radioactive',
    12: 'Corrosive',
    99: 'Start'
}

REQUIRED_SIGN_IDENTIFY_COUNT = 3
SECONDS_UNTIL_RESET = 10

NUMBER_OF_SIGNS_VIEWABLE_AT_ONCE = 2

START_SIGN_ID = 99

marked_signs = []

class Recognise:
    def __init__(self):
        print("IM RUNNING BOSS")
        rospy.init_node('object_recognition_listener', anonymous=True)
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.hazard_marker_pub = rospy.Publisher('/hazards', Marker, queue_size=10)
        self.start_marker_pub = rospy.Publisher('/startMarker', Int32, queue_size=10)
        self.lidar_scan = None
        rospy.Subscriber('/scan', LaserScan, self.lidar_scan_callback)

        # Map Signs to the Count of Confident Instances
        self.confident_sign_detection_within_timer = {}

        rospy.spin()

    def lidar_scan_callback(self, msg):
        self.lidar_scan = msg

    def callback(self, msg):

        self.confident_signs = []

        for i in range(0, len(msg.objects.data), 12):
            object_id = int(msg.objects.data[i])
            confidence = msg.objects.data[i + 1] / 1000

            image_x = msg.objects.data[i + 7]

            # if confidence >= 0.3:
            rospy.loginfo("Object ID: " + str(signage_id[object_id]) + " Confidence: " + str(confidence))
            self.confident_signs.append(object_id)

        # Increment Count of Occurences of Sign
        for sign_id in self.confident_signs:
            if sign_id != -1 and sign_id not in marked_signs:
                if sign_id in self.confident_sign_detection_within_timer:
                    count = self.confident_sign_detection_within_timer[sign_id]
                    self.confident_sign_detection_within_timer[sign_id] = count + 1
                else:
                    # else add sign to confident signs
                    self.confident_sign_detection_within_timer[sign_id] = 1
        
        # Check if any signs have been detected more than 2 times
        for sign_id, count in self.confident_sign_detection_within_timer.items():
            if count >= REQUIRED_SIGN_IDENTIFY_COUNT:
                rospy.loginfo("Sign Confidently Found: (" + str(signage_id[sign_id]) + ", " + str(count) + ")")

                if sign_id == START_SIGN_ID:
                    self.startPosition()
                else:
                    self.otherMarker(sign_id, image_x)
        
        rospy.loginfo(self.confident_sign_detection_within_timer)

    def startPosition(self):
        print("Start sign found")
        startMessage = Int32()
        startMessage.data = 1
        self.start_marker_pub.publish(startMessage)
        print("Start Marker Found")

    def otherMarker(self, sign_id, image_x):
        print("Other Marker Found")
        # Add to List of Marked Signs
        marked_signs.append(sign_id)

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
            hazard_marker = self.create_hazard_marker(sign_id, map_x, map_y, 0)
            self.hazard_marker_pub.publish(hazard_marker)

    def create_hazard_marker(self, marker_id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
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

        marker.lifetime = rospy.Duration()

        marker.id = marker_id

        return marker
    
    def transform_point_to_map_frame(self, point_stamped):
        try:
            target_frame = "map"
            source_frame = point_stamped.header.frame_id
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return transformed_point_stamped.point.x, transformed_point_stamped.point.y, transformed_point_stamped.point.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform object position to map frame")
            return None, None, None

if __name__ == '__main__':
    try:
        recogniser = Recognise()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass