#! /usr/bin/env python3
# Copyright 2022 Joshua Wallace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import glob
import math
import os
import pickle
from random import randint, seed, uniform
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from transforms3d.euler import euler2quat


def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        path = navigator._getPathImpl(initial_pose, goal_pose, planner, use_start=True)
        if path is not None and path.error_code == 0:
            results.append(path)
        else:
            print(planner, 'planner failed to produce the path')
            return results
    return results


def getRandomStart(costmap, max_cost, side_buffer, time_stamp, res):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0] - side_buffer)
        col = randint(side_buffer, costmap.shape[1] - side_buffer)

        if costmap[row, col] < max_cost:
            start.pose.position.x = col * res
            start.pose.position.y = row * res

            yaw = uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            start.pose.orientation.w = quad[0]
            start.pose.orientation.x = quad[1]
            start.pose.orientation.y = quad[2]
            start.pose.orientation.z = quad[3]
            break
    return start


def getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0] - side_buffer)
        col = randint(side_buffer, costmap.shape[1] - side_buffer)

        start_x = start.pose.position.x
        start_y = start.pose.position.y
        goal_x = col * res
        goal_y = row * res
        x_diff = goal_x - start_x
        y_diff = goal_y - start_y
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)

        if costmap[row, col] < max_cost and dist > 3.0:
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y

            yaw = uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            goal.pose.orientation.w = quad[0]
            goal.pose.orientation.x = quad[1]
            goal.pose.orientation.y = quad[2]
            goal.pose.orientation.z = quad[3]
            break
    return goal


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set map to use, other options: 100by100_15, 100by100_10
    # map_path = os.getcwd() + '/' + glob.glob('**/100by100_20.yaml', recursive=True)[0]
    # navigator.changeMap(map_path)
    # time.sleep(2)

    # Get the costmap for start/goal validation
    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)

    planners = ["DEFAULT_Smac2d", "BIDIRECTIONAL_Smac2d", "ALL_DIRECTION_Smac2d", "DEFAULT_SmacHybrid", "BIDIRECTIONAL_SmacHybrid", "ALL_DIRECTION_SmacHybrid", "DEFAULT_SmacLattice", "BIDIRECTIONAL_SmacLattice", "ALL_DIRECTION_SmacLattice"]
    max_cost = 210
    side_buffer = 100
    time_stamp = navigator.get_clock().now().to_msg()
    results = []
    seed(33)

    # # lets get the data 
    # with open('/extra_files/goals.pickle', 'rb') as f:
    #     goals = pickle.load(f)
    
    # start_goals = goals[0]
    # end_goals = goals[1]
    start_goals = []
    end_goals = []

    start_goal = PoseStamped()
    start_goal.header.frame_id = 'map'
    start_goal.header.stamp = time_stamp
    start_goal.pose.position.x = -1.7
    start_goal.pose.position.y = -1.0
    yaw = math.pi/2.0
    quad = euler2quat(0.0, 0.0, yaw)
    start_goal.pose.orientation.w = quad[0]
    start_goal.pose.orientation.x = quad[1]
    start_goal.pose.orientation.y = quad[2]
    start_goal.pose.orientation.z = quad[3]

    end_goal = PoseStamped()
    end_goal.header.frame_id = 'map'
    end_goal.header.stamp = time_stamp
    end_goal.pose.position.x = -1.7
    end_goal.pose.position.y = 1.0
    yaw = math.pi/2.0
    quad = euler2quat(0.0, 0.0, yaw)
    end_goal.pose.orientation.w = quad[0]
    end_goal.pose.orientation.x = quad[1]
    end_goal.pose.orientation.y = quad[2]
    end_goal.pose.orientation.z = quad[3]

    start_goals.append(start_goal)
    end_goals.append(end_goal)

    n = len(start_goals)


    random_pairs = 100
    res = costmap_msg.metadata.resolution
    i = 0
    # while len(results) != random_pairs:
    for i in range(n):
        print('Cycle: ', i, 'out of: ', n)
        # start = getRandomStart(costmap, max_cost, side_buffer, time_stamp, res)
        # goal = getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res)
        start = start_goals[i]
        goal = end_goals[i]
        print('Start', start)
        print('Goal', goal)
        result = getPlannerResults(navigator, start, goal, planners)
        if len(result) == len(planners):
            results.append(result)
            i = i + 1
        else:
            print('One of the planners was invalid')

    print('Write Results...')
    with open(os.getcwd() + '/results.pickle', 'wb+') as f:
        pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/costmap.pickle', 'wb+') as f:
        pickle.dump(costmap_msg, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/planners.pickle', 'wb+') as f:
        pickle.dump(planners, f, pickle.HIGHEST_PROTOCOL)
    print('Write Complete')
    exit(0)


if __name__ == '__main__':
    main()
