#!/usr/bin/env python
import rospy
import time
import copy
from enum import Enum
from math import sqrt, pow, cos, sin
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_point

class GlobalPlanner:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.processLidar, queue_size=1)
        self.gps_sub = rospy.Subscriber("/uav/sensors/gps", PoseStamped, self.processGPS, queue_size=1)

        self.position = Vector3()
        self.grid = OccupancyGrid()
        self.grid.info.width = rospy.get_param("/global_planner_node/map_width", 23)
        self.grid.info.height = rospy.get_param("/global_planner_node/map_height", 23)

        self.dist_to_objects = []
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None

        self.mainloop()

    def processLidar(self, msg):
      self.dist_to_objects = msg.ranges
      self.angle_min = msg.angle_min
      self.angle_max = msg.angle_max
      self.angle_increment = msg.angle_increment
      self.scanArea()
    
    def processGPS(self, msg):
      self.position.x = msg.pose.position.x
      self.position.y = msg.pose.position.y
    
    def scanArea(self):
      curr_angle = self.angle_min
      for dist in self.dist_to_objects:
        if(dist == float('inf')):
          break
        x = cos(curr_angle) * dist
        y = sin(curr_angle) * dist 
        curr_angle += self.angle_increment
        rospy.loginfo("X: " + str(x) + " | Y: " + str(y))

    # The main loop of the function
    def mainloop(self):
      # Set the rate of this loop
      rate = rospy.Rate(20)

      # While ROS is still running
      while not rospy.is_shutdown():

        # Sleep for the remainder of the loop
        rate.sleep()


if __name__ == '__main__':
  rospy.init_node('global_planner_node')
  try:
    ktp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
