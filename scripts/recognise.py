import rospy
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

colors = ["Red", "Green", "Blue", "Yellow", "Orange", "Black", "White"]
values = [[2, 3, 5, 6, 8], [3], [4, 11], [7, 8], [1], [9, 12], [9, 10, 11, 12]]

colours_to_sign_ids = {}
for i in range(len(colors)):
    colours_to_sign_ids[colors[i]] = values[i]

REQUIRED_SIGN_IDENTIFY_COUNT = 2
SECONDS_UNTIL_RESET = 10

NUMBER_OF_SIGNS_VIEWABLE_AT_ONCE = 2

START_SIGN_ID = 99

class Recognise:
    def __init__(self):
        print("IM RUNNING BOSS")
        rospy.init_node('object_recognition_listener', anonymous=True)
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)
        self.image_detected_pub = rospy.Publisher('/image_detected', Bool, queue_size=1)
        self.start_image_pub = rospy.Publisher('/start_image', Bool, queue_size=1)

        # Map Signs to the Count of Confident Instances
        self.confident_sign_detection_within_timer = {}

        rospy.Subscriber('/find_object_2d/red', Image, self.red_callback)
        rospy.Subscriber('/find_object_2d/green', Image, self.green_callback)
        rospy.Subscriber('/find_object_2d/blue', Image, self.blue_callback)

        rospy.spin()

    def timer_callback(self, msg):
        self.reset_signs_identify_count_within_5_seconds('Timer Callback: ' + str(msg))

    def reset_signs_identify_count_within_5_seconds(self, calledFrom):
        self.confident_sign_detection_within_timer = {}
        rospy.loginfo("--- Reset " + calledFrom + " ---")

    def callback(self, msg):

        self.confident_signs = []

        for i in range(0, len(msg.objects.data), 12):
            object_id = int(msg.objects.data[i])
            confidence = msg.objects.data[i + 1] / 1000

            if confidence >= 0.3:
                rospy.loginfo("Object ID: " + str(signage_id[object_id]) + " Confidence: " + str(confidence))
                self.confident_signs.append(object_id)

        # Increment Count of Occurences of Sign
        for sign_id in self.confident_signs:
            if sign_id != -1:
                if sign_id in self.confident_sign_detection_within_timer:
                    count = self.confident_sign_detection_within_timer[sign_id]
                    self.confident_sign_detection_within_timer[sign_id] = count + 1
                else:
                    # else add sign to confident signs
                    self.confident_sign_detection_within_timer[sign_id] = 1
        
        # Check if any signs have been detected more than 2 times
        for sign, count in self.confident_sign_detection_within_timer.items():
            if count >= REQUIRED_SIGN_IDENTIFY_COUNT:
                rospy.loginfo("Sign Confidently Found: (" + str(signage_id[sign]) + ", " + str(count) + ")")
                self.reset_signs_identify_count_within_5_seconds('Num Confident Instances Satisified')

                if sign == START_SIGN_ID:
                    self.startPosition()

                # Print the Sign Name found to have the Highest Confidence
                rospy.loginfo("Sign Confidently Found: (" + str(signage_id[sign]) + ", " + str(count) + ")")

                # Reset Count
                self.reset_signs_identify_count_within_5_seconds('Num Confident Instances Satisified')
        
        rospy.loginfo(self.confident_sign_detection_within_timer)

    def startPosition(self):
        print("Start sign found")
        # self.start_image_pub.publish(Bool(data=True))

    def red_callback(data):
        rospy.loginfo("red callback")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        red_channel = cv_image[:,:,2] # Extract the red channel
        # Process the red channel data here
        rospy.loginfo("-----")
        rospy.loginfo("Found Colour: Red.")
        rospy.loginfo("Could possibly be sign:")
        for sign_id in colours_to_sign_ids[0]:
            rospy.loginfo(sign_id)
        rospy.loginfo("-----")

    def green_callback(data):
        rospy.loginfo("green callback")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        green_channel = cv_image[:,:,1] # Extract the green channel

    def blue_callback(data):
        rospy.loginfo("blue callback")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        blue_channel = cv_image[:,:,0] # Extract the blue channel
        # Process the blue channel data here

if __name__ == '__main__':
    try:
        recogniser = Recognise()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass