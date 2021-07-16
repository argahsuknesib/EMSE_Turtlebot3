#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, Path, GridCells
from move_base_msgs.msg import MoveBaseActionGoal
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import message_filters
import csv


def callback(msg_odom,msg_global_costmap, msg_goal, msg_path, msg_cmd_vel):
    linear_velocity_x = msg_cmd_vel.linear.x
    linear_velocity_y = msg_cmd_vel.linear.y
    linear_velocity_z = msg_cmd_vel.linear.z

    linear_velocity_list = []
    linear_velocity_list.append(linear_velocity_x)
    linear_velocity_list.append(linear_velocity_y)
    linear_velocity_list.append(linear_velocity_z)

    angular_velocity_x = msg_cmd_vel.angular.x
    angular_velocity_y = msg_cmd_vel.angular.y
    angular_velocity_z = msg_cmd_vel.angular.z

    angular_velocity_list = []
    angular_velocity_list.append(angular_velocity_x)
    angular_velocity_list.append(angular_velocity_y)
    angular_velocity_list.append(angular_velocity_z)

    odom_x = msg_odom.pose.pose.position.x
    odom_y = msg_odom.pose.pose.position.y

    odometry_list = []
    odometry_list.append(odom_x)
    odometry_list.append(odom_y)

    global_position_x = msg_global_costmap.info.origin.position.x
    global_position_y = msg_global_costmap.info.origin.position.y
    global_orientation_y = msg_global_costmap.info.origin.position.y
    global_orientation_x = msg_global_costmap.info.origin.position.x

    global_costmap_list = []
    global_costmap_list.append(global_position_x)
    global_costmap_list.append(global_position_y)
    global_costmap_list.append(global_orientation_x)
    global_costmap_list.append(global_orientation_y)

    goal = msg_goal.goal
    global_path = msg_path.poses

    while True:
        with open('/home/whiskygrandee/catkin_ws/src/emse_bot/log/demo-log.csv','w', encoding = 'UTF-8') as f:
            writer = csv.writer(f, delimiter=',', lineterminator= '\n')
            writer.writerow(linear_velocity_list, angular_velocity_list, odometry_list, global_costmap_list, goal, global_path)

rospy.init_node('logger', anonymous=False)

odom_sub = message_filters.Subscriber('/odom', Odometry)
global_costmap_sub = message_filters.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid)
goal_sub = message_filters.Subscriber('/move_base/goal', MoveBaseActionGoal)
path_sub = message_filters.Subscriber('/move_base/DWAPlannerROS/global_plan', Path)
cmd_vel_sub = message_filters.Subscriber('/cmd_vel', Twist)

ts = message_filters.TimeSynchronizer([odom_sub, global_costmap_sub, goal_sub, path_sub, cmd_vel_sub], queue_size=5)
ts.registerCallback(callback)
rospy.spin()