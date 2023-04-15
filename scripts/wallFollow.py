import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

def move_to(x, y, theta):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(theta/2)
    goal.target_pose.pose.orientation.w = math.cos(theta/2)

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def follow_wall(scan):
    global min_distance

    # Define the safe distance to the wall
    safe_distance = 0.6

    # Find the minimum distance to the wall on the left side
    print("LENGTH " ,scan.ranges.length)
    left_ranges = scan.ranges[:90]  # Consider the first 90 readings (270 degrees to 180 degrees)
    left_min_distance = min(left_ranges)

    if left_min_distance < min_distance:
        stop_robot()

    # Calculate the desired angle to keep the robot parallel to the wall
    desired_angle = math.radians(90)

    # Calculate the angle of the closest point on the left
    min_angle_index = left_ranges.index(left_min_distance)
    current_angle = math.radians(min_angle_index)

    # Calculate the new angle to follow the wall
    new_angle = desired_angle - current_angle

    # Calculate the new x, y, and theta to move to
    x = left_min_distance * math.cos(new_angle) + safe_distance * math.sin(new_angle)
    y = left_min_distance * math.sin(new_angle) - safe_distance * math.cos(new_angle)
    theta = new_angle

    # Send the new goal to move_base
    move_to(x, y, theta)

# Replace 'laser_callback' with 'follow_wall' in rospy.Subscriber

def stop_robot():
    global pub
    # Create a Twist message with zero velocities
    stop_twist = Twist()
    stop_twist.linear.x = 0.0
    stop_twist.angular.z = 0.0
    # Publish the message to stop the robot
    pub.publish(stop_twist)

def main():
    global pub

    rospy.init_node('wall_follow')
    
    # Subscribe to the laser scan topic
    rospy.Subscriber('/scan', LaserScan, follow_wall)
    
    # Publish the velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.spin()

