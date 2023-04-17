#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

# Call At about 6 Minutes 15 Seconds
TIME_BEFORE_RETURN = 370

class ReturnRobot(object):
    def __init__(self):
        rospy.init_node('return_robot', anonymous=True)

        self.timer = rospy.Timer(rospy.Duration(TIME_BEFORE_RETURN), self.timercallback)

        # Note: Not Worried about Orientation
        self.start_location_to_return_to = None
        self.start_marker_sub = rospy.Subscriber('/start_travelled_marker', Marker, self.start_marker_callback)

    def start_marker_callback(self, msg):
        # Get Coordinates of Start Pos
        self.start_location_to_return_to = msg.pose.position

    def timercallback(self, msg):
        rospy.loginfo("Time to head back my dudes ...")

        # TODO: Do D* to get back to start location

if __name__ == '__main__':
    try:
        returnRobot = ReturnRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
