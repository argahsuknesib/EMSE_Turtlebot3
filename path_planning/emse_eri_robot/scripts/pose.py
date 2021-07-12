import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

PI = 3.1415926535897
theta = 0

def pose_callback(msg):
    if msg.linear_velocity == 0 and msg.angular_velocity == 0:
        rospy.loginfo("x: %.2f, y: %.2f" % (msg.x, msg.y))
        rospy.loginfo('Orientation in euler - theta:{}'.format(msg.theta))

def move():
    msg.linear.x = FORWARD_SPEED_IN_MPS
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while current_distance < DISTANCE_IN_METERS:
        pub.publish(msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = FORWARD_SPEED_IN_MPS * (t1 - t0)
    msg.linear.x = 0

def turn():
    current_angle = 0
    angular_speed = ROUND_SPEED * 2 * PI / 360
    relative_angle = 45 * 2 * PI / 360
    t0 = rospy.Time.now().to_sec()
    msg.angular.z = abs(angular_speed)

    while current_angle < relative_angle:
        pub.publish(msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)

if __name__ == "__main__":
    FORWARD_SPEED_IN_MPS = 0.5
    DISTANCE_IN_METERS = 1
    ROUND_SPEED = 5
    rospy.init_node("move_turtle")
    pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("turtle1/pose", Pose, pose_callback)
    msg = Twist()
    rate = rospy.Rate(100)
    move()
    turn()
    time.sleep(2)