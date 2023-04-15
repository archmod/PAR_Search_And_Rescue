import rospy
from find_object_2d.msg import ObjectsStamped

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

def callback(msg):    
    # Iterate through the objects in the message
    highest_confidence = 0
    highest_id = -1
    confidence = 0

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

    # Adds delay to the program
    # rospy.sleep(2)    

def main():
    print("IM RUNNING FUCKER")
    rospy.init_node('object_recognition_listener', anonymous=True)
    rospy.Subscriber('/objectsStamped', ObjectsStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
