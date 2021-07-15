#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, Path, GridCells
from move_base_msgs.msg import MoveBaseActionGoal
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from emse_eri_robot_dijkstra import detect_obstacle_space, dijkstra, find_neighbors
from gridviz import GridViz
    

# def print_obstacle(req):
#     costmap = req.costmap_ros
#     width = req.width
#     height = req.height
#     start_index = req.start
#     goal_index = req.goal

#     resolution = 0.2
#     origin = [-7.4, -7.4, 0]
#     viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)
#     path = dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, viz)
    

def cmd_vel_callback(msg):
    linear_velocity_x = msg.linear.x
    linear_velocity_y = msg.linear.y
    linear_velocity_z = msg.linear.z

    angular_velocity_x = msg.angular.x
    angular_velocity_y = msg.angular.y
    angular_velocity_z = msg.angular.z

    linear_velocity_list = []
    linear_velocity_list.append(linear_velocity_x)
    linear_velocity_list.append(linear_velocity_y)
    linear_velocity_list.append(linear_velocity_z)


    angular_velocity_list = []
    angular_velocity_list.append(angular_velocity_x)
    angular_velocity_list.append(angular_velocity_y)
    angular_velocity_list.append(angular_velocity_z)

    return linear_velocity_list, angular_velocity_list


def odom_callback(msg):
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

    odometry_list = []
    odometry_list.append(odom_x)
    odometry_list.append(odom_y)

    return odometry_list

def global_costmap_callback(msg):
    global_position_x = msg.info.origin.position.x
    global_position_y = msg.info.origin.position.y
    global_orientation_y = msg.info.origin.position.y
    global_orientation_x = msg.info.origin.position.x

    global_costmap_list = []
    global_costmap_list.append(global_position_x)
    global_costmap_list.append(global_position_y)
    global_costmap_list.append(global_orientation_x)
    global_costmap_list.append(global_orientation_y)
    
    return global_costmap_list

def goal_callback(msg):
    goal = msg.goal

    return goal

def global_plan_callback(msg):
    global_path = msg.poses

    return global_path

def clock_callback(msg):
    clock_time = msg.clock
    return clock_time

def main():
    rospy.init_node('log_generation_csv')
    # log_generation_service = rospy.Service('/log_generation', PathPlanningPlugin, print_obstacle)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, global_costmap_callback)
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_callback)
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, global_plan_callback)
    rospy.Subscriber('/clock', Clock, clock_callback)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

