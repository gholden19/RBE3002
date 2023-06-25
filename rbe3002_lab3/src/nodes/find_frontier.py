#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, GridCells
from nav_msgs.srv import GetMap, GetPlan
from path_planner import euclidean_distance, grid_to_world, world_to_grid, is_cell_walkable

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
    proxy = rospy.ServiceProxy('map/cpace', GetMap)
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
      
      def cost(self, frontier, x, y):
        """
        ### MODIFY VALUES BASED ON TESTING !!!!!!!!
        """
        dist = euclideanDistance(frontier[0], (x, y))
        if dist < 5:
            dist = 900000000 # effectively infinity
            
        cost = frontier[1] / (1.5 * dist)  # higher distance higher cost
        return frontier
      
      
      def split_four(list):
    """
    """
    groups = []
    cur = []
    for node in list:
        cur.append(node)
        if not float(euclidean_distance(node, next), 1):
            groups.append(cur)
            cur = []
    cur.append(next)
    groups.append(cur)
    
    return groups
    

def split_eight(self, lst):
    """
    """
    groups = []
    cur = []
    for node in list:
        cur.append(node)
        distance = euclidean_distance(node, next)
        if not (float_eq(distance, 1) or float_eq(distance, numpy.sqrt(2)
            groups.append(cur)
            cur = []
    cur.append(next)
    groups.append(cur)
    return groups

def find_centroids(groups):
    """
    """
    list = []
    for group in groups:
        list.append(centroid(group)) 
    
    return list
                                                  
 

