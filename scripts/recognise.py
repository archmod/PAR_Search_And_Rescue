import rospy
from find_object_2d.msg import ObjectsStamped

def callback(msg):
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
    if highest_id == 10:
        print("START SIGN FOUND")



def main():
    print("IM RUNNING FUCKER")
    rospy.init_node('object_recognition_listener', anonymous=True)
    rospy.Subscriber('/objectsStamped', ObjectsStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
