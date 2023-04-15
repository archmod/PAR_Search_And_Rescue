import rospy
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool
from std_msgs.msg import Int32
# from par_search_and_rescue.msg import HazardMarker
from geometry_msgs.msg import PoseStamped
from enum import Enum

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

        rospy.spin()

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


if __name__ == '__main__':
    try:
        hazard_detection_node = HazardDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
