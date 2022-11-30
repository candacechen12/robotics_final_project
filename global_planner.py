# !/usr/bin/env python

# Suhayla Ahmed (sia3yd)
# The purpose of the global_planner is to plan a path to a goal.
# This planning must be done in a way that is cognizant of the unknown map
# and explore in order to find the dog. You may implement the global planner however you like,
# using what you’ve learned from the previous labs.

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from Queue import PriorityQueue
import math


class GlobalPlanner():
  # On node initialization
  def __init__(self):
    self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.processLidar, queue_size=1)
    self.mainloop()

  def processLidar(self, msg):
    rospy.loginfo(msg)

  def plan(self, map_data, drone_position, dog_position):
    self.validate_data(map_data, drone_position, dog_position)
    map_data = self.expand_obstacles(map_data, self.safe_distance)
    frontier = PriorityQueue()
    frontier.put((0, drone_position))
    came_from = {}
    cost_so_far = {}
    came_from[str(drone_position)] = None
    cost_so_far[str(drone_position)] = 0
    final_path = []
    while not frontier.empty():
      current = frontier.get()
      current = list(current[1])
      if current[0] == dog_position[0] and current[1] == dog_position[1]:
          while came_from[str(current)] != None:
              final_path.append(current)
              current = came_from[str(current)]
          final_path.append(current)
          return list(reversed(final_path))
      for next in self.get_neighbors(current, map_data):
        edge_cost = math.sqrt(((current[0] - next[0])**2) + ((current[1] - next[1]) ** 2))
        goal_cost = math.sqrt(((dog_position[0] - next[0])**2) + ((dog_position[1] - next[1]) ** 2))
        new_cost = cost_so_far[str(current)] + edge_cost
        if str(next) not in cost_so_far or new_cost < cost_so_far[str(next)]:
            cost_so_far[str(next)] = new_cost
            priority = new_cost + goal_cost
            frontier.put((priority, next))
            came_from[str(next)] = current

    print("Error: No Path Found")
    return [drone_position]

  # main loop
  def mainloop(self):
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
      pass

    rate.sleep()

if __name__ == '__main__':
  rospy.init_node('global_planner_node')
  try:
    ktp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
