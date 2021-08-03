#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Int32, Float32, Float64, String, Int8
from move_base_msgs.msg import MoveBaseActionGoal
import csv

"""The different message types we wish to get are,
    1. x,y values from Odometry Pose.
    2. x,y values from target MoveBaseActionGoal goal Pose.
    3. Calculated Manhattan Distance.
    4. data[] array from the nav_msgs.msg/Occupancy grid.
    5. Grid Size and Resolution of the Grid.
""" 

class LoggerClass(object):
    def __init__(self):
        pass

data_grid = []
odom_x = 0
odom_y = 0
goal_x = 0
goal_y = 0
manhattan_distance = 0


def odometry_callback(msg):
    global odom_x, odom_y
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    return odom_x, odom_y

def goal_callback(msg):
    global goal_x, goal_y
    goal_x = msg.goal.target_pose.pose.position.x
    goal_y = msg.goal.target_pose.pose.position.y
    return goal_x, goal_y
    

def manhattan_distance_callback(msg):
    global manhattan_distance
    manhattan_distance = msg.distance
    return manhattan_distance

def grid_callback(msg):
    global data_grid
    data_grid = msg.data
    return data_grid

def main():
    subscriber_odom = rospy.Subscriber("/odom", Odometry, odometry_callback)
    subscriber_goal = rospy.Subscriber("/goal", MoveBaseActionGoal, goal_callback)
    subscriber_manhattan = rospy.Subscriber("/manhattan", Int32, manhattan_distance_callback)
    subscriber_grid = rospy.Subscriber("/grid", OccupancyGrid, grid_callback)

    odometry_value = []
    odometry_value.append(odometry_callback)

    goal_value = []
    goal_value.append(goal_callback)

    manhattan_distance_value = manhattan_distance_callback()
    
    grid_data = []
    grid_data = grid_callback()


    with open("/home/whiskygrandee/catkin_ws/src/emse_turtlebot/log/csv-log/log.csv", "w") as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(odometry_value, goal_value, manhattan_distance_value, grid_data)
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

