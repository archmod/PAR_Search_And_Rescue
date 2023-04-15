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

class Recognise:
    def __init__(self):
        print("IM RUNNING BOSS")
        rospy.init_node('object_recognition_listener', anonymous=True)
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.callback)
        self.image_detected_pub = rospy.Publisher('/image_detected', Bool, queue_size=1)
        rospy.spin()

    def callback(self, msg):
        # Iterate through the objects in the message
        highest_confidence = 0
        highest_id = -1

        for i in range(0, len(msg.objects.data), 12):
            object_id = int(msg.objects.data[i])
            confidence = msg.objects.data[i + 1]
            print("Recognized object ID:", object_id, "Confidence:", confidence)
            if confidence > highest_confidence:
                highest_confidence = confidence
                highest_id = object_id
        print("Highest confidence object ID:", highest_id, " Confidence:", highest_confidence)

        # Considers If No Sign is Found
        if highest_id != -1:
            # Print the Sign Name found to have the Highest Confidence
            print(signage_id[highest_id])
            self.image_detected_pub.publish(Bool(data=True))

        if highest_id == 13:
            self.startPosition()

    def startPosition():
        print("Start sign found")

if __name__ == '__main__':
    try:
        recogniser = Recognise()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass