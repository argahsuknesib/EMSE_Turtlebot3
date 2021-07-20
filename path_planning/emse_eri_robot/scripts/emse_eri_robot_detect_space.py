#! /usr/bin/env python
import rospy

def clean_shutdown():
    rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('detect_space', log_level=rospy.INFO, anonymous=False)
    rospy.on_shutdown(clean_shutdown)

    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting Down'), oneshot=True)
    