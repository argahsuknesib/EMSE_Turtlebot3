#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

odom_x = 0
odom_y = 0
goal_x = 0
goal_y = 0


def odom_callback(msg):
    global odom_x, odom_y
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

def goal_callback(msg):
    global goal_x, goal_y
    goal_x = msg.goal.target_pose.pose.position.x
    goal_y = msg.goal.target_pose.pose.position.y

def calculate_distance():
    global goal_x, goal_y, odom_x, odom_y
    manhattan_distance = abs(abs(goal_x - odom_x) - abs(goal_y - odom_y))
    return manhattan_distance

def main():
    rospy.init_node('manhattan_distance')
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goal_callback)
    publisher = rospy.Publisher('/manhattan_distance', Int32, queue_size = 5)
    distance = calculate_distance()
    rospy.loginfo(distance)
    publisher.publish(distance)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass