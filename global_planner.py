#!/usr/bin/env python
import rospy
import time
import copy
from enum import Enum
from math import sqrt, pow
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

        self.mainloop()

    def processLidar(self, msg):
      rospy.loginfo(msg)
    
    def processGPS(self, msg):
      self.position.x = msg.pose.position.x
      self.position.y = msg.pose.position.y
    
    
    
    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(20)

        # While ROS is still running
        while not rospy.is_shutdown():
            pass
        # Sleep for the remainder of the loop
        rate.sleep()


if __name__ == '__main__':
  rospy.init_node('global_planner_node')
  try:
    ktp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
