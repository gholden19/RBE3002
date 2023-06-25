# #!/usr/bin/env python

# from numpy.core.fromnumeric import mean
# from numpy.lib.function_base import disp
# import rospy
# from rospy.names import remap_name
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from geometry_msgs.msg import Twist
# from tf.transformations import euler_from_quaternion
# from std_msgs.msg import Bool
# from final import euclidean_distance, normalize_angle
# import math

# class Sensor:
#     def __init__(self):
#         """
#         Class constructor
#         """
#         rospy.init_node('sensor')

#         self.done_flag = False

#         rospy.sleep(0.05)
#         rospy.Subscriber('/scan', LaserScan, self.sensor_stuff)
        
#         ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
#         self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#         # Subscribe to the amcl 
#         rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_covariance)
#         self.covariance = 900000  # big number

#         self.localized_publisher = rospy.Publisher('/lab4/localized', Bool, queue_size=1)
        
#         # done flag
#         rospy.Subscriber('/lab4/done_flag', Bool, self.update_done)  

#         ### Initialize
#         self.px = 0
#         self.py = 0
#         self.pth = 0
        
#         self.WHEEL_TRACK = 0.16 # [meters] distance between the wheels

#         self.maxspeed_R = 2.84 # [rad/s] max rotational speed
#         self.max_V = 0.22 # [m/s] max linear speed
#         self.delta_V = 0.0025 # [m/s] max change in velocity
#         self.lin_speed = 0.1
#         rospy.sleep(1)  # time to detect node

        
#     def update_done(self, msg): #done flag
#         """
#         """
#         self.done_flag = msg

#     def sensor_stuff(self, msg):
#         tolerance = 0.01
#         offset_range = 4
#         use_point_flag = True
        
#         if not self.done_flag:
#             rospy.loginfo("Not ready!!!")
#             return 

#         else:
#             rospy.loginfo("exploring stuff")
#             msg_time = msg.header.stamp.secs
#             current_time = rospy.get_rostime().secs
          
#             if (current_time - msg_time) > 1:
#                 # rospy.loginfo("Old data. Skipping input")
#                 return

#             else:
#                 self.done_flag = False
#                 bestIndex = 0
#                 bestDist = 0
#                 for i, laser_range in enumerate(msg.ranges):
#                     if laser_range > msg.range_min and laser_range > bestDist and laser_range < msg.range_max:
#                         use_point_flag = True
#                         for offset in range(-offset_range, offset_range+1):
#                             if offset == 0:
#                                 continue
#                             try:
#                                 test_val = msg.ranges[i+offset]
#                             except IndexError:
#                                 test_val = msg.ranges[offset-i]
                            
#                             diff = abs(laser_range - test_val)
#                             #rospy.loginfo("Offset: {} '\t' Diff: {}".format(offset, diff))
#                             if diff > tolerance:
#                                 #rospy.loginfo("Diff too great, skipping value")
#                                 use_point_flag = False
#                                 break
                        
#                         if use_point_flag: 
#                             bestDist = laser_range
#                             bestIndex = i   
                
#                 angle = (bestIndex * msg.angle_increment)
#                 rospy.loginfo("Raw angle: {} radians".format(angle))
#                 angle = normalize_angle(angle)
#                 rospy.loginfo("Angle: {} degrees".format(angle*(180/math.pi)))
#                 rospy.loginfo("Dist: {}m".format(bestDist))
                                
#                 self.rotate(angle)

#                 #rospy.loginfo("driving")
            
#                 #self.smooth_drive(bestDist/4, 0.1)

#                 if self.covariance < 0.001: # too
#                     self.localized_publisher.publish(True)
#                     rospy.loginfo("Localization done")
#                     rospy.signal_shutdown("Pose covariance is {}".format(self.covariance))


#     def update_covariance(self, msg):
#         self.covariance = mean(msg.pose.covariance) #find average of list values
#         rospy.loginfo("Average covariance: {}".format(self.covariance))

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     Sensor().run()
# #
