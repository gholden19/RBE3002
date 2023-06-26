#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetPlan
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
import math
from copy import deepcopy as copy # may be able to get rid of
from nav_msgs.msg import Path, GridCells, Odometry
from geometry_msgs.msg import Twist
import sys
import numpy as np
import cv2
from math import sqrt
from priority_queue import PriorityQueue

def euclidean_distance(a, b):
        """
        """
        distance = sqrt(((b[0] - a[0]) ** 2) + ((b[1] - a[1]) ** 2))
        return distance

def normalize_angle(angle): # makes sure angle is within +/- pi 
    """
    Keeps angle within +/- pi range to normalize it and make it usable in other functions
    :param angle [float] [radians]    Original angle in radians
    :return [float] [radians]         Adjusted angle in radians
    """
    normalize = ((angle + math.pi) % (2 * math.pi)) - math.pi # use mod to reduce angles that are not managable for functions as written to the turtlebot's range
    return normalize

# Master Node :c
class Final:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('final')

        ### Initialize
        self.current_pose = PoseStamped() 
    
        self.px = 0
        self.py = 0
        self.pth = math.pi/2
        
        self.WHEEL_TRACK = 0.16 # [meters] distance between the wheels

        self.maxspeed_R = 2.84 # [rad/s] max rotational speed
        self.max_V = 0.22 # [m/s] max linear speed
        self.delta_V = 0.0025 # [m/s] max change in velocity
        self.lin_speed = 0.1

        self.map_array = np.zeros(1) 

        # Constants used in map filtering and frontier detection
        self.empty = np.uint8(0)
        self.full = np.uint8(255)

        self.map_serv = '/dynamic_map' #if map_source == "dynamic" else '/static_map'

        ## OLD MAP ##
        self.origin = None
       
        self.phase = 1 # initial for state machine
        self.startTime = rospy.get_time()
        
        # --------------------------------------------------------------------------- #
        ### SERVICE ###
        rospy.Service('/frontier', GetPlan, self.find_frontiers)

        # Init service for C-space calculation
        #rospy.Service('map/cspace', GetMap, self.cspace_service)

        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service('plan_path', GetPlan, self.plan_path)

        # --------------------------------------------------------------------------- #
        ### PUBLISHERS ###
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.done_pub = rospy.Publisher('/lab4/done_flag', Bool, queue_size=2) 

        # Nav goal publisher
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # attempting to visualizeeeee
        self.cspace_pub = rospy.Publisher('/map/cspace_visual', GridCells, queue_size=10)

        # Publishes the current goal
        self.goal_pub = rospy.Publisher('/move_base_simple/current_goal', PoseStamped, queue_size=10)

        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.path = rospy.Publisher('/path_planner/path', Path, queue_size=10)
        self.viz_publisher = rospy.Publisher('/frontier_viz', GridCells, queue_size=10)
        # --------------------------------------------------------------------------- #
        ### SUBSCRIBERS ###  

        rospy.Subscriber('/path_planner/path', Path, self.drive_robot)
        rospy.Subscriber('/lab4/position', PoseStamped, self.update_position)
        # rospy.Subscriber('/odom', Odometry, self.update_position)

        rospy.sleep(1)  # Sleep at end of constructor to give ROS time to detect node

## MISC ##

    def float_eq(self, a, b): 
        """"""
        float = abs(a - b) <= 0.001
        return float

    def grid_to_index(self, mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        width = mapdata.map.info.width
        height = mapdata.map.info.height 
        
        if (x > width-1 or x < 0 or y > height-1 or y < 0): # if point is not within the scope of the map
            ## Figure out how to raise error
            raise Exception("This point is outside the bounds of the map.") # throw error to let user know
        else: # if within the scope of the map
            return x + y * width # index of the point

    def index_to_grid(self, mapdata, ind):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate. 
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        
        width = mapdata.map.info.width
        height = mapdata.map.info.height 
        
        if (ind >= width*height or ind < 0): # if point is not within the scope of the map
            
            ## Figure out how to raise error
            raise Exception("This point is outside the bounds of the map.") # throw error to let user know
        else: # if within the scope of the map
            x = ind % width
            y = (ind-x)/width
            return x,y

    def grid_to_world(self, mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        resolution = mapdata.map.info.resolution
        origin = Point(mapdata.map.info.origin.position.x, mapdata.map.info.origin.position.y, 0)
        x_world = (x + 0.5) * resolution + origin.x # 0.5 offset to get to middle of origin cell
        y_world = (y + 0.5) * resolution + origin.y
        return Point(x_world, y_world, 0)

    def world_to_grid(self, mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        resolution = mapdata.map.info.resolution
        origin = Point(mapdata.map.info.origin.position.x, mapdata.map.info.origin.position.y, 0)
        x_grid = int((wp.x - origin.x) / resolution)
        y_grid = int((wp.y - origin.y) / resolution)
        return Point(x_grid, y_grid, 0)

    def path_to_poses(self, mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        new_path = [] # empty list of PoseStamped to convert path into

        for point in path:
            #point = self.grid_to_world(mapdata, point.x, point.y) # use mapdata to make point
            pose = PoseStamped()
            #point = pose.pose.position
            pose.pose.position = point
            new_path.append(pose)

        return new_path

    def is_cell_walkable(self, mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        width = mapdata.map.info.width
        height = mapdata.map.info.height
        if (x > width-1 or x <= 0 or y > height-1 or y <= 0): # point is outside of scope of the map
            return False # not an accessible point

        index = self.grid_to_index(mapdata, x, y) # get index
        wall = 100
        if mapdata.map.data[index] == wall: # check if the given point is within a walkable distance
            return False # not walkable
        else: 
            return True # point is walkable


## DRIVE FUNCTIONS ##
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
        # rospy.loginfo("send speed good"

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
            
        actual_angle = normalize_angle(angle) # normalized to +/- pi range
        goal_angle = normalize_angle(actual_angle + self.pth) # normalized w/in robot's frame

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


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        

        ### REQUIRED CREDIT
        initial_pose = (self.px, self.py, self.pth) # assign initial pose

        # rospy.loginfo("Initial: " + str(initial_pose))
        # convert to euler angles to get theta for goal pose 
        #rospy.loginfo(msg)
        
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]  # convert quaternion to euler angles for goal pose
        (roll, pitch, yaw) = euler_from_quaternion(orientation) # get yaw for the goal pose
        goal_pose = (msg.pose.position.y, msg.pose.position.x, yaw) # (x,y,theta)

        # rospy.loginfo("Goal: " + str(goal_pose))
        # xy offset to find theta and distance
        xy_offset = (goal_pose[0] - initial_pose[0], goal_pose[1] - initial_pose[1])
        # rospy.loginfo(xy_offset)
        th = math.atan2(xy_offset[1], xy_offset[0]) # theta
        # rospy.loginfo(th)
        dist = self.linear_distance(initial_pose, goal_pose) # distance

        # rotate, drive, rotate movement 
        real_angle = normalize_angle(th - initial_pose[2])
        # rospy.loginfo(real_angle)
        self.rotate(real_angle, 1) # turn to line between the two points
        rospy.sleep(1) # set 
        self.smooth_drive(dist, 0.1) # adjust linear speed to drive
        # rospy.sleep(1)
        # self.rotate(goal_pose[2] - th, 1) # final step to goal orientation by turning to final position
        
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

    def angle_diff(self, angle1, angle2): 
        """
        Finds difference in two given angles including accounting for +/- pi continuity issues
        :param angle1 [float] [radians]   First angle
        :param angle2 [float] [radians]   Second angle
        :return [float] [radians]         Difference between the two given angles
        """
        
        angle1 = normalize_angle(angle1) # normalize angles
        angle2 = normalize_angle(angle2)
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
        TOLERANCE = 0.5 # [meters] distance
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
        while self.current_distance((goal_pose[1], goal_pose[0])) > TOLERANCE:
            # rospy.loginfo("Goal Position: " + str(goal_pose))
            # rospy.loginfo("Robot Position: " + str(self.py) + ", " + str(self.px))
            decrease = vel -self.delta_V # max speed decrease
            increase = vel + self.delta_V # max speed increase

            if slow == False:
                # check_distance = self.current_distance(goal_pose)
                if self.current_distance(goal_pose) < adistance:
                    speed = 0 #nce(goal_pose)  start slowing down if within target distance
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

            difference = normalize_angle(goal_pose[2] - self.pth) # error heading 

            self.send_speed(vel, p*difference) # send speed incorporating error calculations and proportional constant
            rospy.sleep(0.1) # delay for robot
        
        # stop robot
        self.send_speed(0,0)

    def drive_robot(self, msg):
        # publish each pose so the robot can actually drive there
      # should skip first node because the robot is already there
      #rospy.loginfo(msg)
      rospy.loginfo("I AM IN THE DRIVE FUNCTION")
      for pose in msg.poses[1:len(msg.poses)/2]:
        rospy.loginfo("Drive Pose: " + str(pose))
        self.go_to(pose)
        
      
      # let user know robot is done driving the path
    #   self.done.publish(True)
    

## MAP ##
    def request_map(self):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
        None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/dynamic_map')
	
        try: # try to request map from server
            proxy = rospy.ServiceProxy('/dynamic_map', GetMap)
            data = proxy()
            rospy.loginfo("successful map request")
        except:
            rospy.loginfo("failed map request")
            data = None
        return data
    
    def calc_cspace(self, mapdata, padding):
        """
        adds padding
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT

        ## Go through each cell in the occupancy grid


        rospy.loginfo("Calculating Cspaceeeee")
        temp_cspace = []  
        new_cspace = list(mapdata.map.data)
        # range of our map x and y values
        for map_point in range(len(mapdata.map.data)):
            
            # checks if obstacle = 100
            if mapdata.map.data[map_point] == 100:
                # Conversion
                map_point_x,map_point_y = self.index_to_grid(mapdata, map_point)
                
                # temporary list of neighbors
                temp_cspace = self.neighbors_of_8(mapdata,map_point_x,map_point_y)
                # Iterates through each point in the temporary list and makes them equal to 100 and then refers back to the next point
                # value in the original list. 
                for neighbor in temp_cspace:
                    if not (neighbor[0] < 0 or neighbor[0] > (mapdata.map.info.width -1) or neighbor[1] < 0 or neighbor[1] > (mapdata.map.info.height -1)): 
                        new_cspace[self.grid_to_index(mapdata, int(neighbor[0]), int(neighbor[1]))] = 100 
                    
        ## Create a GridCells message and publish it
        # TODO
        msg = GridCells()
        
        cells = [] # empty list for message cells

        # loop through all cspace cells, create NEW point object f
        # msg includes walkable cells, if loop above does not say not after if it will include obstacles
        for y in range(0, mapdata.map.info.height):
            
            for x in range(0, mapdata.map.info.width):
                
                if new_cspace[self.grid_to_index(mapdata, x, y)] == 100:
                    
                    wp = self.grid_to_world(mapdata,x,y)
                    cells.append(Point(wp.x, wp.y, 0))
        # input to message
        
        msg.cell_width = mapdata.map.info.resolution
        msg.cell_height = mapdata.map.info.resolution
        msg.cells = cells
        msg.header.frame_id = "map"
        
        #apply time stamp
        time = rospy.rostime.get_rostime()
        msg.header.stamp.secs = time.secs
        msg.header.stamp.nsecs = time.nsecs 
        
        #publish message
        self.cspace_pub.publish(msg)

        ## Return the C-space after applying new cspace to map
        mapdata.map.data = new_cspace
        return mapdata

    # def cspace_service(self, msg):
    #     mapdata = self.request_map()
    #     padding = 1
    #     mapdata = self.calc_cspace(mapdata, padding)
    #     rospy.loginfo("cspace service mode")
    #     return mapdata

    def neighbors_of_4(self, mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        walkable = [] # will be filled with 4 walkabe cells
        for x_offset in [-1, 1]:  # x offset loop
            if self.is_cell_walkable(mapdata, x + x_offset, y):  # check if walkable
                walkable.append((x + x_offset, y))  # add to list of walkable cells

        for y_offset in [-1, 1]:  # y offset loop
            if self.is_cell_walkable(mapdata, x, y + y_offset):  # check if walkable
                walkable.append((x, y + y_offset))  # add neighbor to list of walkable cells

        return walkable

## PATH PLANNER ##
    def path_to_poses(self, mapdata, path):
        """
        """
        
        #rospy.loginfo("current path: {}".format(path))
        newPath = []  # list of PoseStamped

        for node in path:
            rospy.loginfo("Node: " + str(node.x) + ", " + str(node.y))
            point = self.grid_to_world(mapdata, node.x, node.y)  # create the point using the mapdata
            rospy.loginfo("Point: " + str(point.x) + ", " + str(point.y))
            

            pose = PoseStamped()
            pose.pose.position = point
            newPath.append(pose)
          
        return newPath

    def neighbors_of_8(self, mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        walkable = [] # will be filled with 8 walkabe cells
        for y_offset in [-1, 0, 1]:  # y offset loop
            for x_offset in [-1, 0, 1]: # x offset loop
                if(x_offset == 0) and (y_offset ==0): # both are 0
                    continue # skip cell
            
                elif self.is_cell_walkable(mapdata, x + x_offset, y + y_offset): # check neighbor walkable
                    walkable.append((x + x_offset, y + y_offset)) # add to list of walkable cells
        return walkable #list of walkable cells from current location

    # def request_cspace(self):
    #     """
    #     Requests Mapping node for cspace data
    #     :param: padding [int] Number of cells to dialate walls
    #     :return: cspace [OccupancyGrid]: cspace data
    #     """
    #     rospy.loginfo("lab4: Requesting cspace")
    #     rospy.wait_for_service('map/cspace')
    #     try:
    #         proxy = rospy.ServiceProxy('map/cspace', GetMap)
    #         cspace = proxy()
    #         rospy.loginfo("cspace request successful")
    #     except rospy.ServiceException as e:
    #         rospy.logerr("cspace request failed: %s" % e)
    #         cspace = None
    #     return cspace

    def a_star(self, mapdata, start, goal):

        goal = (goal[1], goal[0])

        """
        :param mapdata [OccupancyGrid]: Map data
        :param start   [(int, int)]: Starting point
        :param goal    [(int, int)]: Goal point
        :return path [[(int, int))]: Path, tuple list
        
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Start: " + str(start))
        rospy.loginfo("Goal: " + str(goal))
        frontier = PriorityQueue()  #not yet explored 2 publishers
        frontier.put(start, 0)
        came_from = dict() # already been visited
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        loop_point = None
        cells = [] #empty cell list
        
        # publish to rviz every node we visited
        # get all keys from came_from
        
        while not frontier.empty():
            current = frontier.get()
            # rospy.loginfo("Current: " + str(current))
            # rospy.loginfo("Goal: " + str(goal))
            if current[0] == goal[0] and current[1] == goal[1]:
                rospy.loginfo("BREAKING")
                loop_point = current
                break

            for next in self.neighbors_of_4(mapdata, current[0], current[1]):
                new_cost = cost_so_far[current] + 1 # euclidean_distance(current.x, current.y, next.x, next.y)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost 
                    priority = new_cost + euclidean_distance(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

            msg_astr = GridCells()
            msg_astr.cell_height = mapdata.map.info.resolution
            msg_astr.cell_width = mapdata.map.info.resolution
            msg_astr.header.frame_id = "map"
            points_astr = [] # List of points

            cells_so_far = list(came_from.keys())
            
            for cell in cells_so_far:
                grid_point = Point()
                grid_point.x = cell[0]
                grid_point.y = cell[1]
                grid_point.z = 0
                wrld_point = self.grid_to_world(mapdata, grid_point.x, grid_point.y)
                points_astr.append(wrld_point)
            msg_astr.cells = points_astr
            #self.wavefront.publish(msg_astr)
            loop_point = current
        rospy.loginfo("loop_point: " + str(loop_point))
        path = Path()
        path.header.frame_id = "map"
        points_path = []
        
        # Loops through and looks at every node backwards from the goal
        while True:
            loop_point2 = PoseStamped()
            loop_point2.pose.position.x = loop_point[0]
            # rospy.loginfo("X: " +str(loop_point2.pose.position.x))
            loop_point2.pose.position.y = loop_point[1]
            # rospy.loginfo("Y: " + str(loop_point2.pose.position.y))
            loop_point2.pose.position.z = 0
            wrld_point2 = self.grid_to_world(mapdata, loop_point2.pose.position.x, loop_point2.pose.position.y)
            points_path.append(wrld_point2)
        
            # Once gone through every node stop
            if loop_point == start:
                break
            else:
               loop_point = came_from[loop_point] # Changes loop_point to the next node from the goal
        # rospy.loginfo(points_path)
        # path_pose = self.path_to_poses(mapdata, points_path) # Converts points to poses
        # rospy.loginfo(path_pose)
        # Adds a header to each individual pose in the list
        points_path = self.optimize_path(points_path)
        new = []
        for elem in points_path:
            pose = PoseStamped()
            pose.pose.position = elem
            pose.header.frame_id = "map"
            new.append(pose)
        path.poses = new
        self.path.publish(path)
        rospy.loginfo("Got to End")
        # rospy.loginfo(path.poses)
        return path
        
        #rospy.sleep(0.01)

    def optimize_path(self, path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
	    # atan2 to find angle between vectors
	
        optimized_path = [path[0]] # newpath
        for i, next_node in enumerate(path[1:]): # get index of all nodes except first
            next_node = optimized_path[-1] # last node in new path
            try:
                third_node = path[i+2]
                theta = self.angle_nodes # helper function angle between nodes
                if abs(theta) >= 0.1: # add node to path if ourside of range
                    optimized_path.append(next_node)
            except: # create error
                optimized_path.append(next_node)
                break
        rospy.loginfo(optimized_path)
        return optimized_path
	    
    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT

        path_info = self.path_to_poses(mapdata, path) # poses from path
        msg = Path()
        msg.poses = path_info # give info to path message

        msg.header.frame_id = "path"

        self.path.publish(msg)
        return(msg)


        rospy.loginfo("Returning a Path message")

    def angle_nodes(self, a, b, c):
        """
        """
        angle = math.atan2(c[0] - b[0], c[1] - b[1]) - math.atan2(b[0] - a[0], b[1] - a[1])
        return angle

    def path_from_out_of_bounds(self, mapdata, start, end, iteration):
        """
        """
        path = [start] # Create path including the start cell
        cells = self.get_cells_in_radius(start[0], start[1], iteration) # get the cells at the radius of the current iteration
        rospy.loginfo("Iterations to leave cspace: {}".format(iteration))

        # For all the cells at the current radius
        for cell in cells:
            # Check if any cells are walkable
            if is_cell_walkable(mapdata, cell[0], cell[1]):

                # If it is a valid cell, re-evaluate the pathfinding from this cell
                path.extend(self.a_star(mapdata, cell, end))
                return path

        # If no cell is found to be walkable, iterate again with a bigger radius
        return self.path_from_out_of_bounds(mapdata, start, end, iteration+1)

    def get_cells_in_radius(self, x, y, radius):
        """
        """
 
        cells = []
        # Find all cells within x-offset
        for x_offset in range(x-radius, x+radius+1):
            cells.append((x_offset, y + radius))
            cells.append((x_offset, y - radius))

        # Find all cells within y-offset
        for y_offset in range(y-radius, y+radius+1):
            cells.append((x + radius, y_offset))
            cells.append((x - radius, y_offset))
        return cells

    def plan_path(self, msg):
        
        # get map
        # error, return an empty path
        mapdata = self.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the cspace and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ##  A*
        start = self.world_to_grid(mapdata, msg.start.pose.position)
        goal  = self.world_to_grid(mapdata, msg.goal.pose.position)
        rospy.loginfo("inside service cal")
        path  = self.a_star(cspacedata, start, goal)
        #path = []
        ## Optimize waypoints
        waypoints = self.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)
        #return path


## FIND FRONTIER ##
    def find_centroid(self, contour):
        """
        Finds centroid of the contour. 
        :param region [[int, int]] List of points in a region.
        :return [(int, int)] Centroid of the region (tuple).
        """
        #rospy.loginfo(contours)
        x_c = 0
        y_c = 0

        l = len(contour) # number of points in contour
        for point in contour:
            x_c += point[0][0]
            y_c += point[0][1]
            
        xfinal = x_c / l
        yfinal = y_c / l

        # loop through region and fine euclidean distance of every point
        # save the point with the shortest distance
        # return that as euclidean distance instead of point that is potentially not even a part of the region
        
        return(xfinal, yfinal)

    def find_frontiers(self, cspacedata):
        """
        Takes the [OccupancyGrid] or [EvidenceGrid] and finds the frontiers, returning their centers and lengths 
        :return:    [[((int, int), int)]]     Returns a list of tuples:
        Grid world representation of the centroid (as a tuple) and length of the frontiers
        """
        # -1 unknown
        # 0 ish known / open
        # 255 known / not open . padding
        rospy.loginfo("INSIDE FRONTIER")
        input_data = np.array(cspacedata.map.data)
        
        height = cspacedata.map.info.height
        width = cspacedata.map.info.width
        # map_data = [[0]*cspacedata.map.info.width]*cspacedata.map.info.height # 2D img array
        map_data = []
        for x in range(width):
            map_data.append([])
            for y in range(height):
                map_data[x].append(0)
        

        for i in range(len(input_data)):
            x, y = self.index_to_grid(cspacedata, i)
            map_data[x][y] = input_data[i]
            # if map_data[x][y] != -1:
            #     rospy.loginfo(map_data[x][y])  

        for x in range(width):
            for y in range(height):
                if(map_data[x][y] == -1):
                    map_data[x][y] = 255 # unknown cells
        map_data = np.array(map_data, dtype = np.uint8)
        #rospy.loginfo(map_data)

        # black is walkable, grey is obstacle, white is unknown
        

        #known = np.where(map_data < 5, map_data, 10*map_data)
        known = np.where(map_data == 255, self.full, self.empty)
        open = np.where((map_data < 25) & (map_data >= 0), self.full, self.empty)
        # for i in open:
        #     if i != self.empty:
        #         rospy.loginfo("Found " + str(i))

        #opencv calls 
        known_edges = cv2.Canny(known, 100, 200)
        known_edges = cv2.dilate(known_edges, np.ones((2, 2), np.uint8), iterations=1)

        cv2.imshow('known edge', known_edges)
        cv2.imshow('open', open)
       

        frontiers = []
        for x in range(width):
            frontiers.append([])
            for y in range(height):
                frontiers[x].append(0)

        for x in range(width):
            for y in range(height):
                if(known_edges[x][y] == 255 and open[x][y] == 255):
                    frontiers[x][y] = 255
                else:
                    frontiers[x][y] = 0
        frontiers = np.array(frontiers, dtype = np.uint8)

        cv2.imshow('frontiers', frontiers)
        

        # rospy.loginfo(frontiers)
        # for i in frontiers:
        #     if i != [0]:
        #         rospy.loginfo("Found " + str(i))

        frontiers = cv2.dilate(frontiers, np.ones((4, 4), np.uint8), iterations=1)
        # rospy.loginfo(frontiers)
        cv2.imshow('dilated frontiers', frontiers)
        frontiers = cv2.erode(frontiers, np.ones((3, 3), np.uint8), iterations=1)
        #rospy.loginfo(frontiers)
        # for i in frontiers:
        #     if i != [0]:
        #         rospy.loginfo("Found " + str(i))
        
        cv2.imshow('eroded frontiers', frontiers)
       
        # may need to come back and go through contour stuff 
        _,contours,hierarchy = cv2.findContours(frontiers, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        cv2.drawContours(frontiers, contours, -1, (0,255,0), 3)
        # cv2.waitKey(0)
        # visual_cells = np.argwhere(frontiers[:,:] != 0)
        # visual_points = []

        # for cell in visual_cells:
        #     #grid = self.index_to_grid(cspacedata, cell)
        #     wp = self.grid_to_world(cspacedata, cell[1], cell[0])
        #     #visual_points.append(self.grid_to_world(cspacedata, grid))
        #     visual_points.append(Point(wp.x, wp.y, 0))
        
        # viz_msg = GridCells()
        # viz_msg.header.frame_id = "map"
        # time = rospy.rostime.get_rostime()
        # viz_msg.header.stamp.secs = time.secs
        # viz_msg.header.stamp.nsecs = time.nsecs
        # viz_msg.cells = visual_points
        # viz_msg.cell_height = cspacedata.map.info.resolution
        # viz_msg.cell_width = cspacedata.map.info.resolution
        # self.viz_publisher.publish(viz_msg)

        # rospy.loginfo("contours")
        # rospy.loginfo(len(contours))
        # rospy.loginfo(contours)

        return contours  

    def frontier_cost(self, centroid, arc_length, x, y):
        """
        ### MODIFY VALUES BASED ON TESTING !!!!!!!!
        Calculates the cost of a frontier
        :param frontier:    [((int, int), int)] The Frontier, tuple for centroid then length
        :param x:           [float]             The x location of the robot
        :param y:           [float]             The y location of the robot
        :return:            [float]             The cost of the given frontier, distance from is punished
        """
        
        #rospy.loginfo(frontiersList)

        dist = euclidean_distance(centroid, (x, y))
        if dist < 10:
            dist = 9000000000 # effectively infinity
            
        return dist/arc_length #frontier[1] / (1.2 * dist)  # more distance to frontier, more cost allocated

    def best_centroid(self, msg):
        """
        Returns the centroid of the best frontier as a PathPLan message
        :param msg: 
        :return:
        """
        rospy.loginfo("Entered Best Centroid")
        rospy.sleep(0.1)
        mapdata = self.request_map()
        padding = 3
        mapdata = self.calc_cspace(mapdata, padding)  # update map data
        frontiers = self.find_frontiers(mapdata) # returns contours
        #self.find_centroid(frontiers)

        centroids = []
        arc_lengths = []

        for contour in frontiers:
            centroids.append(self.find_centroid(contour))  
            arc_lengths.append(len(contour))
        # rospy.loginfo(centroids)
        # rospy.loginfo(arc_lengths)

        start_point = self.world_to_grid(mapdata, msg.position)
        if len(frontiers) > 0:  # keep exploring
            lowest_cost = 9999999 # effectively infinity
            lowest_cost_frontier = 0
            for i in range(len(frontiers)):
                #rospy.loginfo(front)
                cost = self.frontier_cost(centroids[i], arc_lengths[i], start_point.x, start_point.y)
                if cost < lowest_cost:
                    lowest_cost = cost
                    lowest_cost_frontier = centroids[i]

            #rospy.loginfo("Best Frontier Found: ({}, {})".format(best[0], best[1]))
            goal_position = self.grid_to_world(mapdata, lowest_cost_frontier[0], lowest_cost_frontier[1])
            return_pose = PoseStamped()
            return_pose.header.frame_id = "map"
            return_pose.pose.position = goal_position

        else:
            rospy.loginfo("In Else Statement")
            return_pose = PoseStamped()
            return_pose.header.frame_id = "sad" # cant find centroid 

        returnmsg = Path()
        returnmsg.poses = [return_pose]
        return returnmsg, mapdata

    def split_four(self, list):
        """
        splits list of grid cells 
        :param lst: [[(float, float)]]      The list of nodes
        :return:    [[[(float, float)]]]    The list of regionsss
        """
        regions = []

        current = []
        for node in list: 
            current.append(node)
            if not float_eq(euclidean_distance(node, next), 1):  #check if not four
                regions.append(current)
                current = []
        
        current.append(next)
        regions.append(current)
            
        return regions 

    def split_eight(self, list):
        """
        split list of cells
        :param lst: [[(float, float)]]      The list of noes
        :return:    [[[(float, float)]]]    The list of regions
        """
        regions = []
        current = []
        for node in list:
            current.append(node)
            dist = euclidean_distance(node, next)
            if not ((float_eq(dist, 1)) or (float_eq(dist, math.sqrt(2)))):  
                regions.append(current)
                current = []
        
        current.append(next)
        regions.append(current)
            
        return regions 

    def run_split(self, list, eight = True): 
        """
        turns list into sections
        :param list: 
        :param eight: [boolean]  
        :return:    list regions
        """
        if eight:
            return split_eight(list)
        else:
            return split_four(list)          
            
    def find_centroids(self, regions):
        """
        centroids for each region in a list
        :param regions: [[[(float, float)]]]    The list of regions
        :return:        [[(float, float)]]      The list of centroids for the regions in the list
        """
        centroids_list = []
        for region in regions:
            centroid_list.append(find_centroid(region)) 
        
        return centroid_list

## OLD MAP ##
    def update_position(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current position information.
        """
        
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y

        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]

        self.current_pose.pose = msg.pose
        
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        # Set theta angle of the robot to yaw value
        self.pth = yaw + (math.pi/2)
        # rospy.loginfo(self.pth)
        
        if not self.origin:
            self.origin = PoseStamped()  # init start as origin
            self.origin.pose.position.x = self.px
            self.origin.pose.position.y = self.py

            quaternion = quaternion_from_euler(0.0, 0.0, self.pth)
            self.origin.pose.orientation.x = quaternion[0]
            self.origin.pose.orientation.y = quaternion[1]
            self.origin.pose.orientation.z = quaternion[2]
            self.origin.pose.orientation.w = quaternion[3]
            self.origin.header.frame_id = "map"

            self.goal = copy(self.origin)  # init goal as origin

    # def run_phases(self, msg):
    #     rospy.loginfo("In run phase!!")
    #     if msg:
    #         if self.phase == 1:

    #             rospy.loginfo("Requesting frontier centroid")
    #             rospy.wait_for_service('/frontier', timeout = 2.0) # change topic back !!!
    #             try:
    #                 proxy = rospy.ServiceProxy('/frontier', GetPlan)
    #                 goal = proxy(self.current_pose, self.current_pose, 0)
    #                 rospy.loginfo("Centroid request successful")
    #             except rospy.ServiceException as e:
    #                 rospy.loginfo("Centroid request Failed: {}".format(e))
    #                 goal = None

    #             if not (goal is None or goal.plan.poses[0].header.frame_id == "sad"): # sad no point found
    #                 rospy.loginfo("send nav goal")
    #                 self.goal_pub.publish(goal.plan.poses[0])
    #             else:  # navigate back to origin, needs to also switch maps
    #                 self.phase = 2
    #                 rospy.loginfo("Phase 1 Complete: going home")
    #                 self.run_phases(True)

    #         elif self.phase == 2:
    #             self.phase = 0
    #             self.goal_pub.publish(self.origin)
            
                
    #         elif self.phase == 0:
    #             rospy.loginfo("Phase 2 Complete: map saved and shuts down")
    #             rospy.loginfo("Mapping Completed in {:.2f} seconds".format(rospy.get_time() - self.startTime))  #timing check

    #         else:
    #             rospy.logerr("Unexpected Lab Phase: {}".format(self.phase))

    def set_goal(self, msg): # add to later
        self.goal = msg
        pass

    def run(self):
        # rospy.loginfo("localization: node ready first ")
        # self.send_speed(0, 0.5)
        # rospy.sleep(1)
        # self.send_speed(0, -0.5)
        # rospy.sleep(1)
        # rospy.loginfo("AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
        # self.done_pub.publish(True)
        while(True):
            msg = self.current_pose.pose
            
            goal, data = self.best_centroid(msg)
            real_msg = self.world_to_grid(data, msg.position)
            # rospy.loginfo(" MSG: " + str(msg.position))
            real_goal = self.world_to_grid(data, goal.poses[0].pose.position)
            rospy.loginfo("NEW Goal: " + str(goal.poses[0].pose.position))
            # rospy.sleep(20)
            # contour = find_frontiers(mapdata)
            # msg = best_centroid(contour)

            rospy.loginfo("Done with cetnroid")

            frontier_msg = self.a_star(data, (real_msg.x, real_msg.y), (real_goal.x, real_goal.y))
            self.drive_robot(frontier_msg)
            rospy.loginfo("got throught drive !!")
            self.send_speed(0, 0)

        rospy.spin()

if __name__ == '__main__':
    Final().run()
    #     if len(sys.argv) < 2:
    #     Map("dynamic").run()
    # else:
    #     Map(sys.argv[1]).run()

    
