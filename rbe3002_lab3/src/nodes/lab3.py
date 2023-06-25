#!/usr/bin/env python
import math
from priority_queue import PriorityQueue
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from locale import normalize
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Lab3:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('lab3')
        # subscribe to 2d nav goal
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.make_plan) 

        # subscribe to odometry from turtlebot
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
       

        # publisher to send points to drive commands, same as lab2 from earlier lab
        self.command_publisher = rospy.Publisher('/lab3/path_points', PoseStamped, queue_size = 10)

        #rospy.Publisher('/lab3', PoseStamped, queue_size = 10)

        self.px = 0
        self.py = 0 
        self.pth = 0

        rospy.sleep(1)


    def make_plan(self, msg):
        """ 
        takes in goal and makes a GetPlan 
        """
        rospy.loginfo("IN MAKE PLAN")
        start = PoseStamped()
        #orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        quant = quaternion_from_euler(0.0, 0.0, self.pth)

        start.pose.position.x = self.px
        start.pose.position.y = self.py
        
        start.pose.orientation.x = quant[0]
        start.pose.orientation.y = quant[1]
        start.pose.orientation.z = quant[2]
        start.pose.orientation.w = quant[3]

        rospy.wait_for_service('plan_path')
        try:
            proxy = rospy.ServiceProxy('plan_path', GetPlan)
            proxy(start, msg, 0)
        except rospy.ServiceException as e:
            rospy.logerr("Path request failed: %s" % e)
        
    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # odometry values set
        self.px = msg.pose.pose.position.x 
        self.py = msg.pose.pose.position.y

        # get info from quaternion message
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list) # convert to euler

        self.pth = yaw # set theta of robot



    def run(self):
        rospy.spin()
    
        
if __name__ == '__main__':
    rospy.loginfo("Lab3 Running")
    Lab3().run()