#!/usr/bin/env python 
import rospy 
from std_msgs import String
from nav_msgs import Odometry
from move_base_msgs.msg import MoveBaseGoal

class ManhattanDistance(object):
    def __init__(self,publisher):
        self._publisher = publisher
        
def odom_callback(msg):
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    return odom_x, odom_y

def goal_callback(msg):
    goal_x = msg.target_pose.pose.position.x
    goal_y = msg.target_pose.pose.position.y
    return goal_x, goal_y

def calculate_distance():
    g_x, g_y = goal_callback()
    o_x, o_y = odom_callback()

    manhattan_distance = abs(abs(g_x - o_x) + abs(g_y - o_y))
    return manhattan_distance

def main():
    rospy.init_node('manhattan_distance')
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/move_base/goal", MoveBaseGoal, goal_callback)
    publisher = rospy.Publisher('/manhattan_distance', String, queue_size = 10)
    distance = calculate_distance()
    publisher.publish(distance)
    rospy.spin()

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
    