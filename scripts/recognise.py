import rospy
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool

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
    13: 'Start'
}

REQUIRED_SIGN_IDENTIFY_COUNT = 2
SECONDS_UNTIL_RESET = 10

class Recognise:
    def __init__(self):
        print("IM RUNNING BOSS")
        rospy.init_node('object_recognition_listener', anonymous=True)
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)
        self.image_detected_pub = rospy.Publisher('/image_detected', Bool, queue_size=1)
        self.start_image_pub = rospy.Publisher('/start_image', Bool, queue_size=1)

        # Map Signs to the Count of Confident Instances
        self.confident_sign_detection_within_timer = {}
        self.timer = rospy.Timer(rospy.Duration(SECONDS_UNTIL_RESET), self.timer_callback)

        rospy.spin()

    def timer_callback(self, msg):
        self.reset_signs_identify_count_within_5_seconds('Timer Callback: ' + str(msg))

    def reset_signs_identify_count_within_5_seconds(self, calledFrom):
        self.confident_sign_detection_within_timer = {}
        rospy.loginfo("--- Reset " + calledFrom + " ---")

    def callback(self, msg):
        # Iterate through the objects in the message
        highest_confidence = 0
        highest_id = -1

        for i in range(0, len(msg.objects.data), 12):
            object_id = int(msg.objects.data[i])
            confidence = msg.objects.data[i + 1]

            if confidence > highest_confidence:
                highest_confidence = confidence
                highest_id = object_id
        
        # Case: If No Sign is Found, Don't Add
        if highest_id != -1:
            # Count Number of Times Sign was Had Highest Confidence
            if highest_id in self.confident_sign_detection_within_timer:
                self.confident_sign_detection_within_timer[highest_id] = self.confident_sign_detection_within_timer.get(highest_id) + 1
            else:
                self.confident_sign_detection_within_timer[highest_id] = 1

        # Check What Signs have High Count
        for sign, count in self.confident_sign_detection_within_timer.items():

            rospy.loginfo("(Sign, Count): (" + str(sign) + ", " + str(count) + ")")

            if count >= REQUIRED_SIGN_IDENTIFY_COUNT:

                # Commented as Causes Stop in Logging Messages
                # self.image_detected_pub.publish(Bool(data=True))

                if highest_id == 13:
                    self.startPosition()

                # Print the Sign Name found to have the Highest Confidence
                rospy.loginfo("Sign Confidently Found: (" + str(signage_id[sign]) + ", " + str(count) + ")")

                # Reset Count
                self.reset_signs_identify_count_within_5_seconds('Num Confident Instances Satisified')

        rospy.loginfo(self.confident_sign_detection_within_timer)

    def startPosition(self):
        print("Start sign found")
        # self.start_image_pub.publish(Bool(data=True))

if __name__ == '__main__':
    try:
        recogniser = Recognise()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass