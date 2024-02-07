from geometry_msgs.msg import PoseStamped
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from transforms3d.euler import quat2euler


def main():
    rclpy.init()

    navigator = BasicNavigator()

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = navigator.get_clock().now().to_msg()
    start.pose.position.x = -2.096
    start.pose.position.y = 0.0
    start.pose.orientation.z = 0.0
    start.pose.orientation.w = 1.0
    start.pose.orientation.x = 0.0
    start.pose.orientation.y = 0.0

    print("Start position: ", start.pose.position.x, ", ", start.pose.position.y, ", ", start.pose.position.z)
    print("Start orientation: ", start.pose.orientation.x, ", ", start.pose.orientation.y, ", ", start.pose.orientation.z, ", ", start.pose.orientation.w)
    navigator.setInitialPose(start)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = 0.5
    goal.pose.position.y = 1.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0

    print("Goal position: ", goal.pose.position.x, ", ", goal.pose.position.y, ", ", goal.pose.position.z)
    print("Goal orientation: ", goal.pose.orientation.x, ", ", goal.pose.orientation.y, ", ", goal.pose.orientation.z, ", ", goal.pose.orientation.w)


    # generate path
    planner = 'SmacHybrid'
    path = navigator._getPathImpl(start, goal, planner, use_start=True)
    if path is not None and path.error_code == 0:
        print("Path found by ", planner)
    else:
        print(planner, 'planner failed to produce the path')
    # print pose and onlyu yaw line by line
    print("Path: ")
    print(path)


if __name__ == '__main__':
    main()