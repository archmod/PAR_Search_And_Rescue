import rospy
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool
from std_msgs.msg import Int32
# from par_search_and_rescue.msg import HazardMarker
from geometry_msgs.msg import PoseStamped
from enum import Enum
from visualization_msgs.msg import Marker

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
    START = 13



class HazardDetection:
    def __init__(self):
        print("IM RUNNING BOSS")
        rospy.init_node('hazard_detection_node', anonymous=True)
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)
        # self.hazard_detected_pub = rospy.Publisher('/hazard_markers_detected', HazardMarker, queue_size=1)
        self.hazard_detected_pub = rospy.Publisher('/hazard_markers_detected', Int32, queue_size=1)
        self.hazard_marker_pub = rospy.Publisher('/hazards', Marker, queue_size=10)
        rospy.spin()

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

    def callback(self, msg):
        # Iterate through the objects in the message
        for i in range(0, len(msg.objects.data), 12):
            object_id = int(msg.objects.data[i])
            confidence = msg.objects.data[i + 1]
            x = msg.objects.data[i + 6]
            y = msg.objects.data[i + 7]
            
            recognized_image = Recognition_Image(object_id)
            print(f"Recognized {recognized_image.name} (ID: {object_id}) with Confidence: {confidence}")
            
            # Publish hazard marker data only if the confidence is above a certain threshold
            if confidence > 0.5 and object_id != Recognition_Image.START.value:
                hazard_marker_id = Int32()
                hazard_marker_id.data = object_id
                self.hazard_detected_pub.publish(hazard_marker_id)
                print(f"Published hazard marker {recognized_image.name} (ID: {object_id}) at (x, y): {x}, {y}")
                hazMarker = self.create_hazard_marker(object_id, x, y, 0.0)
                self.hazard_marker_pub.publish(hazMarker)
                print(f"Published hazard marker {recognized_image.name} (ID: {object_id}) at (x, y): {x}, {y}")
                # rostopic echo /hazard_markers_detected
                # rostopic echo /hazards
                


if __name__ == '__main__':
    try:
        hazard_detection_node = HazardDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
