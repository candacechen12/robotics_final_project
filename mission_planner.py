# !/usr/bin/env python

# Suhayla Ahmed (sia3yd)

import rospy
import time
import copy
from enum import Enum
from use_key.srv import use_key
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point


# A class to keep track of the quadrotors state
class DroneState(Enum):
    EXPLORING = 1
    LOCATING_DOORS = 2
    OPENING_DOORS = 3
    MOVING_TO_WAYPOINT = 4
    UPDATING_MAP = 5


class MissionPlanner:
    def __init__(self):

        self.num_keys_pub = rospy.Publisher("/keys_remaining", int, queue_size=1)

        self.state = DroneState.EXPLORING
        self.num_keys = 4

        self.mainloop()

    def openDoor(self):
        rospy.wait_for_service('use_key')
        try:
            useKey = rospy.ServiceProxy('use_key', use_key)
            resp = useKey(Point(1, 0))
            if (resp == True):
                self.num_keys -= 1
                self.num_keys_pub.publish(self.num_keys)
                print("Successfully used key")
            else:
                print("No more keys left")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # The main loop of the function

    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(20)

        # While ROS is still running
        while not rospy.is_shutdown():
            # Check if the drone is in a moving state
            self.state = DroneState.OPENING_DOORS
            # Publish the position
            self.openDoor()

            # Sleep for the remainder of the loop
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mission_planner_node')
    try:
        ktp = MissionPlanner()
    except rospy.ROSInterruptException:
        pass
