#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from priority_queue import PriorityQueue

class Search:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('search')

        ### Initialize Attributes
        self.px = 0
        self.py = 0
        self.pth = 0
        
        self.goals = PriorityQueue()

        self.phase = 1
        self.startTime = rospy.get_time()

        rospy.sleep(0.05)
        # Subscribes to the position data to track local position
        rospy.Subscriber('/lab4/position', PoseStamped, self.update_position)
        # Subscribes to the done_flag to know when to do the next step
        rospy.Subscriber('/lab4/done_flag', Bool, self.lab)
        # Subscribes to the goal topic to find overall goal
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal)

        rospy.Subscriber('/lab4/localization', Bool, self.update_localized)
        self.localized = False

        # Publishes the current goal
        self.goal_pub = rospy.Publisher('/move_base_simple/current_goal', PoseStamped, queue_size=10)

        rospy.sleep(1)  # Sleep at end of constructor to give ROS time to detect node

    def update_position(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The current position information.
        """
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y

        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]

        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        self.pth = yaw

    def lab(self, msg): # msg goal manager
        if msg and self.goals and self.localized:  # if true
            rospy.loginfo("Sending goal message")
            self.goal_pub.publish(self.goals.get())

    def set_goal(self, msg):
        rospy.loginfo("recieved goal message")
    
        runFlag = True if not self.goals.getList() else False  # if the queue was empty 
            
        self.goals.put(msg)
        
        if runFlag:
            self.lab(True)

    def update_localized(self, msg):
        self.localized = msg

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    Search().run()
