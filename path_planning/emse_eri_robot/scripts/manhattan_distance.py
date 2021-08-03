#!/usr/bin/env python 
import rospy 
from std_msgs import String
from nav_msgs import Odometry
from move_base_msgs.msg import MoveBaseGoal

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
    goal_x = msg.target_pose.pose.position.x
    goal_y = msg.target_pose.pose.position.y

def calculate_distance():
    global goal_x, goal_y, odom_x, odom_y
    manhattan_distance = abs(abs(goal_x - odom_x) - abs(goal_y - odom_y))
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
    