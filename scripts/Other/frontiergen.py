import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose


class Frontier_gen:
    def __init__(self):
        rospy.init_node('frontiergen')
        rospy.Subscriber('/map', OccupancyGrid, self.process_grid)
        self.pub = rospy.Publisher('generate/frontier', PoseArray, queue_size=10)

    def process_grid(self, msg):
        print("message")

        grid_criteria = (-1 , 100)
        data = msg.data
        width = msg.info.width
        height = msg.info.height

        print(f"height: {height}, width: {width}")

        grid = []
        for i in range(height):
            row = []
            for j in range(width):
                row.append(data[i * width + j])
            grid.append(row)

        frontier = []

        for i, _ in enumerate(grid):
            for j, _ in enumerate(grid[i]):
                #is unknown
                if grid[i][j] == -1:
                    if i > 0 and grid[i - 1][j] not in grid_criteria: # up
                        pose = Pose()
                        pose.position.x = j
                        pose.position.y = i - 1
                        pose.orientation.w = 1.0
                        frontier.append(pose)

                    if i < height - 1 and grid[i + 1][j] not in grid_criteria: # down
                        pose = Pose()
                        pose.position.x = j
                        pose.position.y = i + 1
                        pose.orientation.w = 1.0
                        frontier.append(pose)

                    if j > 0 and grid[i][j - 1] not in grid_criteria: # left
                        pose = Pose()
                        pose.position.x = j - 1
                        pose.position.y = i
                        pose.orientation.w = 1.0
                        frontier.append(pose)

                    if j < width - 1 and grid[i][j + 1] not in grid_criteria: # right
                        pose = Pose()
                        pose.position.x = j + 1
                        pose.position.y = i
                        pose.orientation.w = 1.0
                        frontier.append(pose)

        pose_array = PoseArray()
        pose_array.poses = frontier

        print("publishing")
        self.pub.publish(pose_array)



if __name__ == "__main__":
    try:
        frontier_gen = Frontier_gen()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass