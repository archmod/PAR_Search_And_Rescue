#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def drive():
    # Drive publisher
    pub_drive = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    # Paramaters
    speed = rospy.get_param("~speed", 0.5)
    max_duration = rospy.get_param("~duration", 3)
    t_start = rospy.Time.now()
    stop = False

    rospy.loginfo("Rotating robot at speed %lf for %lf seconds", speed, max_duration)

    # Periodically publish the drive command to ensure drive continues
    rate = rospy.Rate(10)
    while (not stop) and (not rospy.is_shutdown()):
        cmd.angular.z = speed
        pub_drive.publish(cmd)
        rate.sleep()
        
        # Check time
        t_now = rospy.Time.now()
        if t_now - t_start > rospy.Duration(max_duration):
        	stop = True

    # Ensure a stop command is sent
    rospy.loginfo("Rotating finished - stopping robot")
    cmd.angular.z = 0
    pub_drive.publish(cmd)

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_rotate', anonymous=True)
        drive()
    except rospy.ROSInterruptException:
        pass
