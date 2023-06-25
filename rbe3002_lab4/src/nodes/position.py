#!/usr/bin/env python2

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from rospy.timer import sleep

# The node exists to understand the pose of the robot based on either odom or amcl
# allows the rest of the odometry and positioning code to be resued

class Position:
    def __init__(self, source):
        rospy.init_node("position")

        if source == "amcl":
            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_handler)
        else:
            rospy.Subscriber('/odom', Odometry, self.pose_handler)

        self.position_publisher = rospy.Publisher('/lab4/position', PoseStamped, queue_size=10)

        rospy.sleep(0.5)
        rospy.loginfo("Positon node ready, using {} for position".format(source))

    def pose_handler(self, msg):
        pos_msg = PoseStamped()
        pos_msg.pose = msg.pose.pose
        pos_msg.header = msg.header
        self.position_publisher.publish(pos_msg)

    def run(self):
        rospy.spin()

if __name__=="__main__":
    # handle command line argument and default to using odom if left blank
    if len(sys.argv) < 2:
        Position("odom").run()
    else:
        Position(sys.argv[1]).run()
