#!/usr/bin/env python2
from locale import normalize
import rospy
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
 
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        
        # CHANGE FROM .go_to TO .arc_to HERE
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to) 
  
        ### Initialize
        self.px = 0
        self.py = 0
        self.pth = 0
        self.WHEEL_TRACK = 0.16 # [meters] distance between the wheels

        self.maxspeed_R = 2.84 # [rad/s] max rotational speed
        self.max_V = 0.22 # [m/s] max linear speed
        self.delta_V = 0.0025 # [m/s] max change in velocity

        rospy.sleep(1) # time for ROS to detect node
        

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()

        # linear velocity
        msg_cmd_vel.linear.x = linear_speed # turtlebot moves forward on x axis 
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed #turtlebot rotates about z

        ### Publish the message
        self.cmd_vel.publish(msg_cmd_vel)
        ##rospy.loginfo("send speed good")

    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        #rospy.loginfo("In Drive")
        goal_pose = (self.px + math.cos(self.pth) * distance, self.py + math.sin(self.pth) * distance, self.pth)
        
        TOLERANCE = 0.05 # [meters] distance
        p = 1 # find proportional coefficient for correcting angle
        while self.current_distance(goal_pose) > TOLERANCE:
            #self.send_speed(linear_speed, 0)
            diff = self.normalize_angle(goal_pose[2] -self.pth) # heading error
            self.send_speed(linear_speed, p *diff) # apply heading correction
            rospy.sleep(0.05) # time for values to update

        self.send_speed(0,0) #stop robot


    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        # save direction for later
        if angle < 0:
            direct = -1
        else:
            direct = 1 
            
        actual_angle = self.normalize_angle(angle) # normalized to +/- pi range
        goal_angle = self.normalize_angle(actual_angle + self.pth) # normalized w/in robot's frame

        TOLERANCE = 0.01 # [radians] distance
        p = 1.1 # proportional coefficient for rotation (ELI says to make higher)
        while abs(self.angle_diff(self.pth, goal_angle)) > TOLERANCE: # while abs value is outside of tolerance value
            difference = abs(self.angle_diff(self.pth, goal_angle)) # error
            rspeed = p* difference 
            # correcting any speeds too high for the robot
            if aspeed > self.maxspeed_R: 
                aspeed = self.maxspeed_R
            # correcting to slow down the closer the robot gets to the goal angle
            if aspeed > rspeed:
                aspeed = rspeed
           
            self.send_speed(0, aspeed*direct) # send speed and incorporate direction saved at the beginning
            rospy.sleep(0.01)
        
        # stob robot
        self.send_speed(0,0)

        #correctedAngle = self.normalize_angle(angle)
        # goal_pose = (self.normalize_angle(angle + self.pth))
        # TOLERANCE = 0.5 

        # if aspeed > self.maxspeed_R:
        #     aspeed = self.maxspeed_R

        # dir = 0
        # if angle > 0:
        #     dir = 1
        # elif angle < 0:
        #     dir = -1
        
        # while abs(goal_pose - self.pth) >= TOLERANCE:
        #     if abs(goal_pose - self.pth) <= 2 * TOLERANCE:
        #         self.send_speed(0, dir * aspeed/2)
        #     else:
        #         self.send_speed(0, dir * aspeed)
        #     rospy.sleep(0.05)
        # self.send_speed(0,0)
        
    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        

        ### REQUIRED CREDIT
        initial_pose = (self.px, self.py, self.pth) # assign initial pose

        # convert to euler angles to get theta for goal pose 
        #rospy.loginfo(msg)
        
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]  # convert quaternion to euler angles for goal pose
        (roll, pitch, yaw) = euler_from_quaternion(orientation) # get yaw for the goal pose
        goal_pose = (msg.pose.position.x, msg.pose.position.y, yaw) # (x,y,theta)

        # xy offset to find theta and distance
        xy_offset = (goal_pose[0] - initial_pose[0], goal_pose[1] - initial_pose[1])
        th = math.atan2(xy_offset[1], xy_offset[0]) # theta
        dist = self.linear_distance(initial_pose, goal_pose) # distance

        # rotate, drive, rotate movement 
        real_angle = self.normalize_angle(th - initial_pose[2])
        self.rotate(real_angle, 1) # turn to line between the two points
        rospy.sleep(1) # set 
        self.drive(dist, 0.1) # adjust linear speed to drive
        rospy.sleep(1)
        self.rotate(goal_pose[2] - th, 1) # final step to goal orientation by turning to final position


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



    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        vel = 0.1 # [m/s] arbitrary value for velocity, adjust as needed
        initial_pose = (self.px, self.py, self.pth) # starting pose
        
        # convert quanterion to euler to get yaw and assign goal pose
        orientation = [position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z, position.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        goal_pose = (position.pose.position.x, position.pose.position.y, yaw)
        
        # xy_offset and theta between initial and goal points
        xy_off = (goal_pose[0] - initial_pose[0], goal_pose[1] - initial_pose[1]) # Initial endgoal comparison (ICC)
        th = math.atan2(xy_off[1], xy_off[0])
        #rospy.loginfo(th)
       
       # finds center point of circle reference frame as well as radius by finding and manipulating the circle eq
       # ICC in this setion (also something we were told could be an issue, we would probably have to change stuff within circle_eq)
        circle = self.circle_eq(initial_pose, goal_pose, self.pth)
        a_arc = vel / circle[1] # angular velocity for traveling in arc

         # correct direction (turning left vs right)
        if (self.normalize_angle(th) - self.normalize_angle(initial_pose[2])) > 0:
            a_arc = a_arc
        else:
            a_arc = -a_arc
        #rospy.loginfo(a_arc)
        
        # not technically a problem but maybe delete this section as this is where the robot is deciding to go backwards, it can probably work without this by making a bigger rotation initially
        # getting rid of this may make it easier to work on tolerance and rotate
        if abs(th - initial_pose[2]) > (math.pi/2):
            vel = -vel
            a_arc = -a_arc
       
        TOLERANCE = 0.12 # [m] 
        # i would play with tolerance and timing because sometimes it is missing this queue below and that is our problem 
        self.send_speed(vel, a_arc)
        while self.current_distance(goal_pose) > TOLERANCE: # driving until w/ in tolerence 
            rospy.sleep(0.5) #maybe try 0.5

        self.send_speed(0,0) # stop
        self.rotate(goal_pose[2] - self.pth, a_arc) # rotating to final position 
           # again maybe go back to rotate to see if we can improve anything there because this takes a while too
           # i am not sure if both the tolernace and the final rotate are having issues but i think its both of them so that might be something to ask SAs more about if correcting one of them first is easier 
        #self.send_speed(0,0) # stop

        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        initial_pose = (self.px, self.py, self.pth) # assign initial pose
        goal_pose = (self.px + math.cos(self.pth)*distance, self.py + math.sin(self.pth)*distance, self.pth) # assign goal pose

        p = 1 # proportional constant to correct angle
        TOLERANCE = 0.01 # [meters] distance
        vel = 0
        speed = linear_speed # goal speed
        adistance = 0 # [m] for acceleration
        slow = False # robot slowing down / decelerating 

        # keeps target speed within turtlebot's accepted value (both directions maximum)
        if (speed > self.max_V):
            speed = self.max_V
        else:
            if (speed < -self.max_V):
                speed = -self.max_V
            else:
                speed = speed
                
       # try to prevent spinning out of control and passing the goal angle by adjusting velocity
        while self.current_distance(goal_pose) > TOLERANCE:
            decrease = vel -self.delt_aV # max speed decrease
            increase = vel + self.delta_V # max speed increase

            if slow == False:
                # check_distance = self.current_distance(goal_pose)
                if self.current_distance(goal_pose) < adistance:
                    speed = 0 # start slowing down if within target distance
                    slow = True
                elif (vel != linear_speed and self.current_distance(goal_pose) < (distance / 2)):
                    speed = 0 # start slowing down if halfway to goal
                    slow = True

            if speed > increase: # speed above max acceleration speed increase
                vel = increase # set to max to avoid setting speed too high for the robot
                adistance = self.current_distance(initial_pose) # update distance
            elif speed < decrease: # speed below decrease speed step
                vel = decrease # set velocity to max decrease speed
            else: # velocity is at target speed 
                vel = speed # no change

            difference = self.normalize_angle(goal_pose[2] - self.pth) # error heading 

            self.send_speed(vel, p*difference) # send speed incorporating error calculations and proportional constant
            rospy.sleep(0.1) # delay for robot
        
        # stop robot
        self.send_speed(0,0)


    def current_distance(self, point):
        """
        Calculate distance from a point to the current position, using linear_distance.
        :param point [(float float)] [(m m)] Point in space
        :return [float] [m]                  Distance between current postiion of turtlebot and the point
        """
        distance = self.linear_distance((self.px, self.py), point)
        return distance
    
    def linear_distance(self, point1, point2):
        """
        Calculate distance between two points
        :param point1 [(float, float)] [(m,m)]    One point
        :param point2 [(float, float)] [(m,m)]    Second point
        :return [float] [m]                        Distance between the two points
        """
        hyp = math.sqrt(math.pow(point2[0] - point1[0], 2) + math.pow(point2[1] - point1[1], 2))
        return hyp

    def normalize_angle(self, angle): # makes sure angle is within +/- pi 
        """
        Keeps angle within +/- pi range to normalize it and make it usable in other functions
        :param angle [float] [radians]    Original angle in radians
        :return [float] [radians]         Adjusted angle in radians
        """
        normalize = ((angle + math.pi) % (2 * math.pi)) - math.pi # use mod to reduce angles that are not managable for functions as written to the turtlebot's range
        return normalize
       

    def angle_diff(self, angle1, angle2): 
        """
        Finds difference in two given angles including accounting for +/- pi continuity issues
        :param angle1 [float] [radians]   First angle
        :param angle2 [float] [radians]   Second angle
        :return [float] [radians]         Difference between the two given angles
        """
        
        angle1 = self.normalize_angle(angle1) # normalize angles
        angle2 = self.normalize_angle(angle2)
        diff = angle1 - angle2 # difference between angles
        #rospy.loginfo("angle 1")
        #rospy.loginfo(angle1)
        #rospy.loginfo("angle 2")
        #rospy.loginfo(angle2)
        if diff > math.pi:
            diff = diff - 2*math.pi
        elif diff < -math.pi:
            diff = diff + 2*math.pi
        return diff # corrected angle difference

    
    def circle_eq(self, p1, p2, tan_angle): # p1 and p2 are two points on the circle
        """
        Finds circle equation for circle that intersects the two given points and is tangent to line with the given angle passing through the first point
        :param point1 [(float, float)] [(m,m)]            First point
        :param point2 [(float, float)] [(m,m)]            Second point
        :return [((float, float), float)] [((m,m),m)]     Circle's center point (x,y) and nradius 
        """
        
        # Finds the circle equation from two points and the tangent angle between them using math from https://math.stackexchange.com/questions/2464364/how-can-i-find-the-equation-of-a-circle-given-two-points-and-a-tangent
        tan_result = math.tan(tan_angle)
        if tan_result == 0:
            slope = 1e20 # effectively infinity
        else:
            slope = -1 / tan_result # slope is perpenducular bisector slope
       
        # circle eq: ax + by = c
        a1 = 2*(p1[0]-p2[0]) # 0 index is x
        b1 = 2*(p1[1]-p2[1]) # 1 index is y
        c1 = math.pow(p1[0], 2) - math.pow(p2[0], 2) + math.pow(p1[1], 2) - math.pow(p2[1], 2)
       
        a2 = -slope
        b2 = 1
        c2 = p1[1] - slope*p1[0] 

        # matrices in numpy from https://www.geeksforgeeks.org/how-to-inverse-a-matrix-using-numpy/
        a_final = numpy.array([[a1,b1],[a2,b2]])
        b_final = numpy.array([c1,c2])

        c_final = numpy.linalg.inv(a_final).dot(b_final) # a_final^(-1) * b_final = c_final solving (x,y) for center of the circle

        r = math.sqrt(math.pow(c_final[0] - p1[0], 2) + math.pow(c_final[1] - p1[1], 2))
        # r = self.linear_distance(p1, c_final)
        return ((c_final[0], c_final[1]), r)


    def run(self):
        #rospy.spin()
        #self.smooth_drive(1, 0.2) # 1 [m] forward @ speed 0.2 [m/s]
        #self.smooth_drive(-0.20, -0.2) # 1 [m] backwards @ speed 0.2 [m/s]
        #self.rotate (math.pi/2, 1.0) 
        #self.rotate (-math.pi/2, 1.0)
        #self.rotate ((3*math.pi/2), 1.0)
        
        # square code
        # self.drive(1, 0.2)
        # self.rotate(math.pi/2, 1.0)
        # self.drive(1, 0.2)
        # self.rotate(math.pi/2, 1.0)
        # self.drive(1, 0.2)
        # self.rotate(math.pi/2, 1.0)
        # self.drive(1, 0.2)
        # self.rotate(math.pi/2, 1.0)
    
if __name__ == '__main__':
    Lab2().run()
    
