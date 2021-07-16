#! /usr/bin/env python
import rospy
from nav_msgs.msg import  Odometry, OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseActionGoal
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import csv

linear_velocity_list = []
angular_velocity_list = []
odometry_list = []
global_costmap_list = []
goal_list = []
global_plan_list = []
clock_list = []

def cmd_vel_callback(msg):
    linear_velocity_x = msg.linear.x
    linear_velocity_y = msg.linear.y
    linear_velocity_z = msg.linear.z

    angular_velocity_x = msg.angular.x
    angular_velocity_y = msg.angular.y
    angular_velocity_z = msg.angular.z

    global linear_velocity_list
    global angular_velocity_list

    linear_velocity_list.append(linear_velocity_x)
    linear_velocity_list.append(linear_velocity_y)
    linear_velocity_list.append(linear_velocity_z)
    angular_velocity_list.append(angular_velocity_x)
    angular_velocity_list.append(angular_velocity_y)
    angular_velocity_list.append(angular_velocity_z)


def odom_callback(msg):
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    global odometry_list
    odometry_list.append(odom_x)
    odometry_list.append(odom_y)

def global_costmap_callback(msg):
    global_position_x = msg.info.origin.position.x
    global_position_y = msg.info.origin.position.y
    global_orientation_y = msg.info.origin.position.y
    global_orientation_x = msg.info.origin.position.x
    global global_costmap_list
    global_costmap_list.append(global_position_x)
    global_costmap_list.append(global_position_y)
    global_costmap_list.append(global_orientation_x)
    global_costmap_list.append(global_orientation_y)


def goal_callback(msg):
    global_path = msg.goal
    global goal_list
    goal_list.append(global_path)

def global_plan_callback(msg):
    global_path = msg.poses
    global global_plan_list
    global_plan_list.append(global_path)

def clock_callback(msg):
    clock_time = msg.clock
    global clock_list
    clock_list.append(clock_time)


def main():
    rospy.init_node('log_generation_csv', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, global_costmap_callback)
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_callback)
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, global_plan_callback)
    rospy.Subscriber('/clock', Clock, clock_callback)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    with open('/home/whiskygrandee/catkin_ws/src/emse_bot/log/demo-log.csv','w', encoding = 'UTF-8') as f:
        writer = csv.writer(f, delimiter=',', lineterminator= '\n')
        writer.writerow(linear_velocity_list)
        writer.writerow(angular_velocity_list)
        writer.writerow(odometry_list)
        writer.writerow(global_costmap_list)
        writer.writerow(goal_list)
        writer.writerow(global_plan_list)
        writer.writerow(clock_list)
    rospy.spin()

if __name__ == '__main__':
    main()
