from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import glob
import time
import os
import numpy as np
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat

# from transforms3d.euler import euler2quat

def main():
    rclpy.init()
    navigator = BasicNavigator()
    # Set map to useç
    # map_path = os.getcwd() + '/' + glob.glob('**/willow-2010-02-18-0.10.yaml', recursive=True)[0]
    # navigator.changeMap(map_path)
    # time.sleep(2)
    # print("Map changed suceessfully")
    # costmap_msg = navigator.getGlobalCostmap()
    # costmap = np.asarray(costmap_msg.data)
    # costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)
    #  # Set initial pose
    # row = 10
    # col = 10
    # res = costmap_msg.metadata.resolution
    # print(res)
    initial_pose = PoseStamped()
    yaw = 1.11209
    quad = euler2quat(0.0, 0.0, yaw)
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 27.9804 #col * res
    initial_pose.pose.position.y = 1.93814 #row * res
    initial_pose.pose.orientation.w = quad[0]
    initial_pose.pose.orientation.x = quad[1]
    initial_pose.pose.orientation.y = quad[2]
    initial_pose.pose.orientation.z = quad[3]
    navigator.setInitialPose(initial_pose)


    # Set goal pose
    row = 100
    col = 100
    goal_pose = PoseStamped()
    yaw = 2.90514
    quad = euler2quat(0.0, 0.0, yaw)
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 21.2687#col * res
    goal_pose.pose.position.y = 54.3685 #row * res
    goal_pose.pose.orientation.w = quad[0]
    goal_pose.pose.orientation.x = quad[1]
    goal_pose.pose.orientation.y = quad[2]
    goal_pose.pose.orientation.z = quad[3]
    # navigator.setInitialPose(goal_pose)

    ## get path 
    planner = 'GridBased'
    path = navigator._getPathImpl(initial_pose, goal_pose, planner_id=planner, use_start=True)
    if path is None and path.error_code != 0:
        print(planner, "planner failed to produce the path")
if __name__ == '__main__':
    main()
