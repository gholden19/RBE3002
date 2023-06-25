#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan


class Lab3:

    def __init__(self):
        """
        Class constructor
        """
        # Create a node to manage the path planner and any related communications not directly handled
        rospy.init_node("lab3")
        rospy.loginfo("Starting lab3 node")

        # Subscribe to the 2D nav goal mesage from mapping node
        rospy.Subscriber('/move_base_simple/current_goal', PoseStamped, self.make_plan)

        # Subscribe to the turtlebot position
        rospy.Subscriber('/lab4/position', PoseStamped, self.update_position)

        self.current = PoseStamped()  # store the current robot pose

        rospy.sleep(1)  # Sleep to let ROS figure out this node exists

    def make_plan(self, msg):
        """
        GET FROM GITHUB
        Takes in a goal position and generates a GetPlan request to find a path to the goal
        Uses the path to send commands to the robot to navigate to the goal.
        :param msg  [PoseStamped] The goal pose of the path
        """

        # Request the plan_path service from the path planner
        rospy.loginfo("Requesting path")
        rospy.wait_for_service('plan_path')
        try:
            # Send the request
            proxy = rospy.ServiceProxy('plan_path', GetPlan)
            proxy(self.current, msg, 0)
            rospy.loginfo("Path request success")
        except rospy.ServiceException as e:
            # If there is a fail, log the error and set the path to empty
            rospy.logerr("Path request failed: %s" % e)

    def update_position(self, msg):
        """
       
        """
        self.current = msg

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    Lab3().run()
