#!/usr/bin/env python

from asyncore import loop
from cmath import cos
from email.header import Header
import math
import numpy as np
import cv2
from os import path
#from msvcrt import open_osfhandle
from turtle import st, width
from priority_queue import PriorityQueue
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
	
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service('plan_path', GetPlan, self.plan_path)
        
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher('/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.path_pub = rospy.Publisher('/path', Path, queue_size = 10)
        self.wavefront = rospy.Publisher('/wavefront', GridCells, queue_size = 10)



        
        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")
        



    @staticmethod
    def grid_to_index(mapdata, x, y):
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


    pass
    @staticmethod
    def index_to_grid(mapdata, ind):
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

    pass


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        ### REQUIRED CREDIT
        distance = math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2)) # calculating distances between two points using x and y coordinates
        return distance
        pass
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
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
        pass


        
    @staticmethod
    def world_to_grid(mapdata, wp):
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


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        new_path = [] # empty list of PoseStamped to convert path into

        for point in path:
            #point = PathPlanner.grid_to_world(mapdata, point.x, point.y) # use mapdata to make point
            pose = PoseStamped()
            #point = pose.pose.position
            pose.pose.position = point
            new_path.append(pose)

        return new_path
    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
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

        index = PathPlanner.grid_to_index(mapdata, x, y) # get index
        walkable = 0
        if mapdata.map.data[index] != walkable: # check if the given point is within a walkable distance
            return False # not walkable
        else: 
            return True # point is walkable


               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
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
            if PathPlanner.is_cell_walkable(mapdata, x + x_offset, y):  # check if walkable
                walkable.append((x + x_offset, y))  # add to list of walkable cells

        for y_offset in [-1, 1]:  # y offset loop
            if PathPlanner.is_cell_walkable(mapdata, x, y + y_offset):  # check if walkable
                walkable.append((x, y + y_offset))  # add neighbor to list of walkable cells

        return walkable

    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
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
            
                elif PathPlanner.is_cell_walkable(mapdata, x + x_offset, y + y_offset): # check neighbor walkable
                    walkable.append((x + x_offset, y + y_offset)) # add to list of walkable cells
        return walkable #list of walkable cells from current location

        pass
    
    
    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
        None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/static_map')
	
        try: # try to request map from server
            proxy = rospy.ServiceProxy('/static_map', GetMap)
            data = proxy()
            rospy.loginfo("successful map request")
        except:
            rospy.loginfo("failed map request")
            data = None
        return data



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        # TODO

        # set to original grid, will change -- for sizing purposes 

        temp_cspace = []  
        new_cspace = list(mapdata.map.data)

        rospy.loginfo(len(mapdata.map.data))

        length = len(mapdata.map.data)
        i = 0
        while i < length:

            # For a point in the range of our map x and y values
            for map_point in range(len(mapdata.map.data)):
                
                # Checks if one of those points is an obstacle set it to 100 and then add padding to every point that is = to 100
                if mapdata.map.data[map_point] == 100:
                    # Conversion
                    map_point_x,map_point_y = PathPlanner.index_to_grid(mapdata, map_point)
                    
                    # Adds neighbor values to a temporary list
                    temp_cspace = PathPlanner.neighbors_of_8(mapdata,map_point_x,map_point_y)
                    # Iterates through each point in the temporary list and makes them equal to 100 and then refers back to the next point
                    # value in the original list. 
                    for neighbor in temp_cspace:
                        if not (neighbor[0] < 0 or neighbor[0] > (mapdata.map.info.width -1) or neighbor[1] < 0 or neighbor[1] > (mapdata.map.info.height -1)): 
                            new_cspace[PathPlanner.grid_to_index(mapdata, int(neighbor[0]), int(neighbor[1]))] = 100 
            i += 1
                    

        # check if each cell is walkable when looping through all of the cells on the map
        # for y in range(1,(mapdata.map.info.height -1)): # within scope of map on y axis
        #   
        #   for x in range(1, (mapdata.map.info.width - 1)): # within scope of map on x axis
        #         current_cell = PathPlanner.is_cell_walkable(mapdata, x, y) # checks if current cell is walkable
        #         adjacent_cell = True # is walkable
        #         rospy.loginfo("prinying x y")
        #         rospy.loginfo(x)
        #         rospy.loginfo(y) 
        #         # if not walkable set all cells in range of padding to value of 100
                # apply change to new cspace grid so modified cells are not checked
                # if not (current_cell and adjacent_cell):
                #     for ypadding in range(padding, padding + 1):
                #         for xpadding in range(padding, padding + 1):
                #             #cells out of scope of map
                #             rospy.loginfo("here")
                #             #rospy.loginfo(ypadding)
                #             newx = x + xpadding
                #             newy = y + ypadding
                #             # if PathPlanner.grid_to_index(mapdata, int(newx),  int(newx)) < ((len(new_cspace)) - 1):
                            
                            #     if (PathPlanner.is_cell_walkable == False):
                            #         rospy.loginfo("Error")
                            #     else:
                            #         rospy.loginfo(PathPlanner.grid_to_index(mapdata, int(newx), int(newy)))
                           # new_cspace[PathPlanner.grid_to_index(mapdata, int(newx), int(newy))] = 100
                            
                            
                             
                            


            ## Create a GridCells message and publish it
            # TODO
            rospy.loginfo("SZFVLKAHNFSLUVHLSUD")
            msg = GridCells()
            rospy.loginfo("next")
            cells = [] # empty list for message cells

            # loop through all cspace cells, create NEW point object for each cell
            # msg includes walkable cells, if loop above does not say not after if it will include obstacles
            for y in range(0, mapdata.map.info.height):
                rospy.loginfo(y)
                for x in range(0, mapdata.map.info.width):
                    rospy.loginfo(x)
                    if new_cspace[PathPlanner.grid_to_index(mapdata, x, y)] == 100:
                        rospy.loginfo("if grid")
                        wp = PathPlanner.grid_to_world(mapdata,x,y)
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
            self.cspace.publish(msg)

            ## Return the C-space after applying new cspace to map
            mapdata.map.data = new_cspace
            return mapdata

    ## Credit: red blob A*
    # def heuristic(a,b):
    #     return abs(a.x - b.x) + abs(a.y, b.y)
    
    def a_star(self, mapdata, start, goal):
        """
        :param mapdata [OccupancyGrid]: Map data
        :param start   [(int, int)]: Starting point
        :param goal    [(int, int)]: Goal point
        :return path [[(int, int))]: Path, tuple list
        
        """
        ### REQUIRED CREDIT
        #rospy.loginfo("Got to Astarrrrrrrrrr")
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

            if current == goal:
                break

            for next in self.neighbors_of_4(mapdata,current[0],current[1]):
                new_cost = cost_so_far[current] + 1 # self.euclidean_distance(current.x, current.y, next.x, next.y)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost 
                    priority = new_cost + self.euclidean_distance(goal[0], goal[1], next[0], next[1])
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
                wrld_point = PathPlanner.grid_to_world(mapdata, grid_point.x, grid_point.y)
                points_astr.append(wrld_point)
            msg_astr.cells = points_astr
            #self.wavefront.publish(msg_astr)
            loop_point = current

        path = Path()
        path.header.frame_id = "map"
        points_path = []
        
        # Loops through and looks at every node backwards from the goal
        while True:
            loop_point2 = PoseStamped()
            loop_point2.pose.position.x = loop_point[0]
            loop_point2.pose.position.y = loop_point[1]
            loop_point2.pose.position.z = 0
            wrld_point2 = PathPlanner.grid_to_world(mapdata, loop_point2.pose.position.x, loop_point2.pose.position.y)
            points_path.append(wrld_point2)
        
            # Once gone through every node stop
            if loop_point == start:
                break
            else:
               loop_point = came_from[loop_point] # Changes loop_point to the next node from the goal

        path_pose = self.path_to_poses(mapdata, points_path) # Converts points to poses
        
        # Adds a header to each individual pose in the list
        new = []
        for elem in path_pose:
            elem.header.frame_id = "map"
            new.append(elem)
        path.poses = new
        self.path_pub.publish(path)
        rospy.loginfo("Got to End")
        
        #rospy.sleep(0.01)

    
    @staticmethod
    def optimize_path(path):
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
                theta = PathPlanner.angle_nodes # helper function angle between nodes
                if abs(theta) >= 0.1: # add node to path if ourside of range
                    optimized_path.append(next_node)
            except: # create error
                optimized_path.append(next_node)
                break
        return optimized_path
	
        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT

        path_info = PathPlanner.path_to_poses(mapdata, path) # poses from path
        msg = Path()
        msg.poses = path_info # give info to path message

        msg.header.frame_id = "path"

        self.path.publish(msg)
        return(msg)


        rospy.loginfo("Returning a Path message")


        
    def plan_path(self, msg):
        
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        rospy.loginfo("inside service cal")
        path  = self.a_star(cspacedata, start, goal)
        #path = []
        ## Optimize waypoints
        #waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        #return self.path_to_message(mapdata, waypoints)
        return path 

    def find_centroid(self, region):
        """
        Finds centroid of the frontier. 
        :param region [[int, int]] List of points in a region.
        :return [(int, int)] Centroid of the region (tuple).
        """
        
        x_c = 0
        y_c = 0
        
        l = len(region)
        
        for gridcell in region:
            x_c += gridcell[0]
            y_c += gridcell[1]
            
        xfinal = x_c / l
        yfinal = y_c / l
        
        # loop through region and fine euclidean distance of every point
        # save the point with the shortest distance
        # return that as euclidean distance instead of point that is potentially not even a part of the region
        
        return(xfinal, yfinal)

    def retrieve_cspace(self, padding):
        """
        Requests cspace data from map?? node.
        :param padding [int] Amount of cells to add to obstacles.
        :return cspace [OccupancyGrid] cspace data
        """
        
        rospy.wait_for_service('map/cspace')
        try:
            proxy = rospy.ServiceProxy('map/cspace', GetMap)
            data = proxy()
            rospy.loginfo("successful cspace retrieval")
            
            self.map_array = np.array(data.map.data)
            self.map_array.shape = (data.map.info.height, data.map.info.width)
            
            np.savetxt("map.csv", self.map_array, delimiter = ",")
            
        except rospy.ServiceException as e:
            rospy.logerr("cspace")
                

    def find_frontiers(self, cspacedata):
        """
        Takes the [OccupancyGrid] or [EvidenceGrid] and finds the frontiers, returning their centers and lengths 
        :return:    [[((int, int), int)]]     Returns a list of tuples:
        Grid world representation of the centroid (as a tuple) and length of the frontiers
        """
        map_data = self.map_array

        known = np.where(map_data == -1, self.empty, self.full)
        open = np.where((map_data < 25) & (map_data >= 0), self.full, self.empty)

        known_edges = cv2.Canny(known, 100, 200)
        known_edges = cv2.dilate(known_edges, np.ones((8, 8), np.uint8), iterations=1)
        frontiers = np.where((known_edges == 255) & (open == 255), self.full, self.empty)
        frontiers = cv2.dilate(frontiers, np.ones((2, 2), np.uint8), iterations=1)
       

        frontiers = cv2.erode(frontiers, np.ones((3, 3), np.uint8), iterations=1)

        contours, hierarchy = cv2.findContours(frontiers, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        visual_cells = np.argwhere(frontiers[:,:] != 0)
        visual_points = []

        for cell in visual_cells:
            wp = self.grid_to_world(cspacedata, cell[1], cell[0])
            visual_points.append(Point(wp.x, wp.y, 0))
        
        viz_msg = GridCells()
        viz_msg.header.frame_id = "map"
        time = rospy.rostime.get_rostime()
        viz_msg.header.stamp.secs = time.secs
        viz_msg.header.stamp.nsecs = time.nsecs
        viz_msg.cells = visual_points
        viz_msg.cell_height = cspacedata.map.info.resolution
        viz_msg.cell_width = cspacedata.map.info.resolution
        self.viz_publisher.publish(viz_msg)


        frontiersList = []  # list of frontier centroids and lengths, 

        for frontier in contours:
            length = cv2.arcLength(frontier, 1)
            if length > 15:  # ignore small frontiers
                try:
                    M = cv2.moments(frontier)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    center = (cX, cY)
                    frontiersList.append((center, int(length)))
                except ZeroDivisionError:
                    rospy.logerr("There was Division by 0")
                
        return frontiersList

    @staticmethod
    def angle_nodes(a, b, c):
        """
        Finds the angle between two points about another using atan2
        :param a [(x,y)] The first point
        :param b [(x,y)] The pivot point
        :param c [(x,y)] The second point
        :return  [int]   The angle in radians
        """
        angle = math.atan2(c[0] - b[0], c[1] - b[1]) - math.atan2(b[0] - a[0], b[1] - a[1])
        return angle

    def run(self): 
        data = self.request_map()
        # rospy.loginfo("REEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
        # rospy.loginfo(data)
        # self.a_star(data, (5,5), (35,10))
        # rospy.sleep(5)
        #self.a_star(data, (20,10), (6,2))
        #self.plan_path(GetPlan)
        while True:
            self.find_frontiers(self.a_star(data))
            rospy.sleep(1)
        rospy.spin() # Do nothing forever

	


if __name__ == '__main__':
    PathPlanner().run()
