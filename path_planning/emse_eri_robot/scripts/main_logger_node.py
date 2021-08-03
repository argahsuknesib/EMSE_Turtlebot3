#! /usr/bin/env python
import rospy
import csv
from std_msgs.msg import String
from nav_msgs import Odometry, OccupancyGrid

def callback_odom(msg):
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

def callback_local_costmap(msg):
    data = []
    data = msg.data

def callback_distance(msg):
    distance = msg.distance

def main():
    rospy.init_node('logger_node')
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback_local_costmap)
    rospy.Subscriber("/manhattan_distance", String, callback_distance)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass