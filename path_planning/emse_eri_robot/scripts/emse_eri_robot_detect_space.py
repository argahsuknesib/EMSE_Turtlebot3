#! /usr/bin/env python
import rospy 
from std_msgs.msg import String
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from gridviz import GridViz
import emse_eri_robot_astar_solution_server
import emse_eri_robot_gbfs_solution_server
import emse_eri_robot_dijkstra

def print_path():
    path_dijkstra = emse_eri_robot_dijkstra.make_plan()
    path_astar = emse_eri_robot_astar_solution_server.make_plan()
    path_gbfs = emse_eri_robot_gbfs_solution_server.make_plan()

def space_path(req):



def clean_shutdown():
    rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('detect_space', anonymous=False)
    make_plan_service = rospy.Service('/move_base/SrvClientPlugin/make_plan', PathPlanningPlugin, space_path)
    

