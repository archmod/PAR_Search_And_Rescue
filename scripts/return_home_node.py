import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class ReturnHomeNode:
    def __init__(self, duration=360):
        rospy.init_node('return_home_node')
        self.endTime = -1
        self.duration = duration
        self.returnHomePublisher = rospy.Publisher('/returnHome', Int32, queue_size=1)
        self.homePose = None
        
        # Initialize MoveBase client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Listen for the start marker to start the timer
        rospy.Subscriber('/startMarker', Int32, self.SetStartTime)
        # Listen for the home position
        rospy.Subscriber('/start_position', Marker, self.SetHomePose)

    def run(self):
        while not rospy.is_shutdown():
            if rospy.get_time() > self.endTime and self.endTime != -1:
                print("Start to return home")
                self.returnHomePublisher.publish(1)
                rospy.sleep(1)
                self.ReturnHome()
            else:
                self.returnHomePublisher.publish(0)
            rospy.sleep(1)
    
    def ReturnHome(self):
        if self.homePose:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.homePose

            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            print("Time to return home")
        else:
            print("Home pose not set, cannot return home.")
        
    
    def SetStartTime(self, msg):
        print("Return home node got start marker!")
        if self.endTime == -1:
            self.endTime = rospy.get_time() + self.duration
    
    def SetHomePose(self, marker):
        if self.homePose is None:
            self.homePose = marker.pose
    
if __name__ == '__main__':
    try:
        rospy.init_node('return_home_node')
        return_home_node = ReturnHomeNode(330)
        return_home_node.run()
    except rospy.ROSInterruptException:
        pass
